#pragma once
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <atomic>
#include <sstream>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <holydrone_autotune/brct_trial.hpp>
#include <condition_variable>
#include <mutex>
#include <cstring>

namespace brct {
// One worker owns transactions. ROS callbacks only deliver fresh FCU PARAM_VALUE replies.
// PID gains are REAL32; never reinterpret integer parameters as floats.
class ParameterLink {
  ros::Publisher tx_;
  ros::Subscriber rx_;
  ros::CallbackQueue reply_queue_;
  ros::AsyncSpinner reply_spinner_{1, &reply_queue_};
  std::atomic<uint64_t> frames_{0}, replies_{0}, wrong_source_{0}, invalid_{0}, unrelated_{0}, old_stamp_{0};
  std::mutex mutex_;
  std::condition_variable changed_;
  int target_system_ = 1, target_component_ = 1;
  std::string expected_;
  ros::Time requested_;
  uint64_t generation_ = 0;
  float value_ = 0;
  uint8_t type_ = 0;

  void receive(const mavros_msgs::Mavlink::ConstPtr& msg) {
    ++frames_;
    if (msg->msgid != MAVLINK_MSG_ID_PARAM_VALUE) return;
    ++replies_;
    if (msg->sysid != target_system_ || msg->compid != target_component_) { ++wrong_source_; return; }
    if (msg->framing_status != mavros_msgs::Mavlink::FRAMING_OK ||
        msg->len < MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN ||
        msg->payload64.size()*8 < msg->len || msg->payload64.size() > 33) { ++invalid_; return; }
    mavlink_message_t packet{};
    packet.msgid = msg->msgid; packet.len = msg->len;
    std::memcpy(packet.payload64, msg->payload64.data(), msg->len);
    mavlink_param_value_t param{}; mavlink_msg_param_value_decode(&packet, &param);
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::string(param.param_id, strnlen(param.param_id,16)) != expected_) { ++unrelated_; return; }
    if (!msg->header.stamp.isZero() && msg->header.stamp < requested_) { ++old_stamp_; return; }
    value_ = param.param_value; type_ = param.param_type; ++generation_; changed_.notify_one();
  }
  void send(const mavlink_message_t& packet) {
    mavros_msgs::Mavlink msg;
    msg.header.stamp = ros::Time::now(); msg.framing_status = msg.FRAMING_OK;
    msg.magic = packet.magic; msg.len = packet.len; msg.seq = packet.seq;
    msg.sysid = packet.sysid; msg.compid = packet.compid; msg.msgid = packet.msgid;
    msg.checksum = packet.checksum; msg.incompat_flags = packet.incompat_flags; msg.compat_flags = packet.compat_flags;
    msg.payload64.resize((packet.len+7)/8);
    std::memcpy(msg.payload64.data(), packet.payload64, packet.len);
    tx_.publish(msg);
  }
 public:
  ~ParameterLink() { rx_.shutdown(); reply_spinner_.stop(); }
  void setup(ros::NodeHandle& nh, const std::string& mavros) {
    nh.param(mavros+"/target_system_id", target_system_, 1);
    nh.param(mavros+"/target_component_id", target_component_, 1);
    if (target_system_ < 1 || target_system_ > 255 || target_component_ < 1 || target_component_ > 255)
      throw std::runtime_error("invalid MAVROS target IDs");
    // MAVROS raw bridge is /mavlink by default; allow remapping for namespaced instances.
    tx_ = nh.advertise<mavros_msgs::Mavlink>("/mavlink/to", 10);
    // Parameter replies must not wait behind control, event logging or IMU callbacks.
    ros::NodeHandle replies(nh); replies.setCallbackQueue(&reply_queue_);
    rx_ = replies.subscribe("/mavlink/from", 100, &ParameterLink::receive, this);
    reply_spinner_.start();
  }
  double exchange(const std::string& name, bool writing = false, double desired = 0) {
    if (name.empty() || name.size() > 16) throw std::runtime_error("invalid parameter name: "+name);
    const auto connect_until = ros::WallTime::now()+ros::WallDuration(1);
    while (ros::ok() && (tx_.getNumSubscribers() == 0 || rx_.getNumPublishers() == 0) && ros::WallTime::now() < connect_until)
      ros::WallDuration(.01).sleep();
    if (tx_.getNumSubscribers() == 0 || rx_.getNumPublishers() == 0)
      throw std::runtime_error("MAVROS raw bridge topics are not connected");
    std::string diagnostic;
    char id[16]{}; std::memcpy(id,name.data(),name.size());
    for (int attempt=0; attempt<3 && ros::ok(); ++attempt) {
      const uint64_t frames=frames_, replies=replies_, source=wrong_source_, invalid=invalid_,
                     unrelated=unrelated_, old=old_stamp_;
      std::unique_lock<std::mutex> lock(mutex_);
      expected_ = name; requested_ = ros::Time::now(); const auto before = generation_;
      mavlink_message_t packet{};
      // Separate component from MAVROS (240); replies must come from the configured FCU.
      if (writing) mavlink_msg_param_set_pack(255, 191, &packet, target_system_, target_component_, id,
                                             static_cast<float>(desired), MAV_PARAM_TYPE_REAL32);
      else mavlink_msg_param_request_read_pack(255, 191, &packet, target_system_, target_component_, id, -1);
      send(packet);
      const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(1);
      auto seen = before;
      while (ros::ok() && std::chrono::steady_clock::now() < deadline && changed_.wait_until(lock, deadline, [&]{return generation_ != seen;})) {
        seen = generation_;
        if (type_ != MAV_PARAM_TYPE_REAL32 || !std::isfinite(value_)) {
          expected_.clear(); throw std::runtime_error("invalid FCU parameter type/value: "+name);
        }
        if (!writing || brct::matches(value_,desired)) { expected_.clear(); return value_; }
      }
      std::ostringstream info;
      info << "target=" << target_system_ << "." << target_component_
           << " bridge_subscribers=" << tx_.getNumSubscribers()
           << " raw_rx=" << frames_-frames << " param_rx=" << replies_-replies
           << " wrong_source=" << wrong_source_-source << " invalid=" << invalid_-invalid
           << " other_parameter=" << unrelated_-unrelated << " old_stamp=" << old_stamp_-old
           << " matched=" << generation_-before;
      if (generation_ != before) info << " last_value=" << value_ << " expected=" << desired;
      diagnostic = info.str(); expected_.clear();
      lock.unlock();
      ROS_WARN("Parameter %s %s confirmation timeout (%d/3): %s", name.c_str(),
               writing ? "write" : "read", attempt+1, diagnostic.c_str());
    }
    throw std::runtime_error(std::string(writing ? "write confirmation timeout: " : "read timeout: ")+name+"; "+diagnostic);
  }
};
}
