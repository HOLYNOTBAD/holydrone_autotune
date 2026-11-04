/**
 * @file brct_autotune_fsm.cpp
 * @brief 管理 PX4 角速度环候选参数的加载、读回验证和自动顺序激励。
 *
 * @details
 * 节点通过 MAVROS 参数服务异步写入候选参数，并持续向
 * `setpoint_raw/attitude` 发布角速度与高度闭环推力，每个脉冲后用角速度闭环短暂回中。
 * 试验仅在 FCU 状态和运动反馈新鲜且处于 Offboard 时启动；手动退出 Offboard 时中止试验。
 * 节点不会解锁或起飞；全部候选的 low/high 完成后请求 POSCTL，不自动选优或恢复 baseline。
 *
 * ROS 接口：
 * - 订阅 `<mavros_namespace>/state`、`timesync_status` 和 `setpoint_raw/target_attitude`；
 * - 发布 `<mavros_namespace>/setpoint_raw/attitude` 和 `/pid_experiment/event`；
 * - 提供 `/load_candidate`、`/run_trial`、`/stop_trial`、`/restore_baseline`、
 *   `/next_candidate` 和 `/get_experiment_status` 服务；
 * - 私有参数 `~config`、`~output` 为必填，`~mavros_namespace` 默认为 `/mavros`。
 */

/* ROS 与消息接口 */
#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/TimesyncStatus.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/SetMode.h>
#include <holydrone_autotune/recenter_control.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <holydrone_autotune/settling_gate.hpp>
#include <std_srvs/Trigger.h>
#include <holydrone_autotune/ExperimentCommand.h>
#include <holydrone_autotune/ExperimentEvent.h>
#include <holydrone_autotune/brct_trial.hpp>
#include <holydrone_autotune/altitude_control.hpp>

/* 配置与文件系统 */
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

/* 标准库 */
#include <chrono>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <thread>

using Parameters = std::map<std::string, double>;
using Command = holydrone_autotune::ExperimentCommand;
using Event = holydrone_autotune::ExperimentEvent;
using Clock = std::chrono::steady_clock;

/** @brief 返回不受 ROS 仿真时间跳变影响的单调时间，单位为 s。 */
static double steady() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

/** @brief 将参数映射序列化为事件消息使用的单行 YAML。 */
static std::string dump(const Parameters& p) { YAML::Emitter e; e << YAML::Flow << p; return e.c_str(); }

/** @brief 一组具有唯一标识的 PX4 角速度环候选参数。 */
struct Candidate { std::string id; Parameters values; };

/** @brief 后台参数写入及 FCU 读回验证的结果。 */
struct ApplyResult { bool ok = false; Parameters readback; std::string detail; };

/* ============================ 试验状态机 ============================ */

/**
 * @brief 协调候选参数、激励时序、Offboard 保护和会话记录。
 *
 * @note ROS 回调由单线程 `ros::spin()` 串行执行；只有 `apply()` 在后台线程运行，
 *       该线程通过 `std::future` 返回结果，不直接修改状态机字段。
 */
class Autotune {
  /** @brief 全局与私有命名空间的 ROS 句柄。 */
  ros::NodeHandle nh_, private_{"~"};
  /** @brief 角速度指令和试验事件发布器。 */
  ros::Publisher command_, events_;
  /** @brief FCU 状态和 MAVROS 时间同步订阅器。 */
  ros::Subscriber vehicle_, timesync_, thrust_feedback_, pose_sub_, velocity_sub_, imu_sub_;
  /** @brief 使用墙钟驱动的 200 Hz 指令与状态机定时器，不随 `/clock` 暂停。 */
  ros::WallTimer timer_;
  /** @brief 保持所有服务端对象存活。 */
  std::vector<ros::ServiceServer> services_;
  /** @brief 后台参数操作；仅由主回调线程轮询并取回结果。 */
  std::future<ApplyResult> pending_;
  /** @brief 逐事件和时间同步日志；写入失败会关闭节点。 */
  std::ofstream journal_, sync_journal_;
  /** @brief 配置中的恢复基线与当前已选择候选。 */
  Candidate baseline_, current_;
  /** @brief 按配置顺序保存的试验候选，不含 baseline。 */
  std::vector<Candidate> candidates_;
  /** @brief 最近一次从 FCU 强制刷新后逐项读回的参数。 */
  Parameters readback_;
  /** @brief 会话标识、FRD 激励轴、状态、错误信息和 MAVROS 命名空间。 */
  std::string experiment_, axis_, state_ = "IDLE", error_, mavros_;
  /** @brief 归一化推力、时序（s）、两档幅值（deg/s）和单调启动时刻（s）。 */
  double thrust_ = 0, settle_, duration_, low_, high_, amplitude_ = 0, started_ = 0;
  // Position 悬停时跟踪实际推力指令，进入 Offboard 后冻结，退出后重新采集。
  double thrust_received_ = 0;
  const double thrust_timeout_ = 0.5;
  bool position_armed_ = false, thrust_locked_ = false;
  ros::Time thrust_stamp_, position_since_;
  geometry_msgs::Point position_now_, origin_;
  geometry_msgs::Vector3 velocity_now_;
  double yaw_ = 0, origin_yaw_ = 0, cos_tilt_ = 1;
  double pose_received_ = 0, velocity_received_ = 0;
  ros::Time pose_stamp_, velocity_stamp_, control_stamp_;
  bool origin_locked_ = false, pulse_active_ = false, safety_exit_ = false;
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  brct::RecenterControl recenter_;
  int pulse_index_ = 0;
  double stage_started_ = 0, sent_thrust_ = 0;
  brct::SettlingGate settling_;
  double angular_speed_ = 0, imu_received_ = 0;
  double clock_lead_tolerance_ = 0;
  ros::Time imu_stamp_;
  double angular_speed_tolerance_ = .2, angular_feedback_timeout_ = .1;
  // 反馈有效性窗口，仅用于接收、控制发布和回中判定，不触发自动模式切换。
  const double feedback_timeout_ = .5;
  double recovery_timeout_ = 8, position_tolerance_ = .15, speed_tolerance_ = .3, horizontal_tolerance_ = .75;
  double tilt_tolerance_deg_ = 5, yaw_tolerance_deg_ = 5;
  brct::AltitudeControl altitude_;
  /** @brief 最近 FCU 状态的单调接收时间及允许陈旧时间，单位为 s。 */
  double state_received_ = 0, state_timeout_ = 2.0;
  /** @brief Offboard 状态缓存，以及当前参数是否通过 FCU 读回确认。 */
  bool offboard_ = false, confirmed_ = false;
  // 一次进入触发完整列表；中止后再次进入从首项重试，完成后本会话不重复。
  bool sequence_active_ = false, sequence_done_ = false, sequence_restart_ = false;
  bool high_level_ = false, waiting_for_exit_ = false;
  double last_exit_request_ = 0;
  std::future<bool> exit_request_;
  /** @brief 当前候选、激励阶段和重复轮次索引；-1 表示尚未进入。 */
  int repetitions_, candidate_index_ = -1, phase_ = -1, repetition_ = -1;
  /** @brief 本次节点会话内单调递增的试验编号。 */
  uint32_t trial_ = 0;

  /** @brief 读取节点启动所需的私有 ROS 参数。 */
  void loadStartupParameters(std::string& config, std::string& output) {
    if (!private_.getParam("config", config) || !private_.getParam("output", output))
      throw std::runtime_error("config and unique output directory required");
    private_.param<std::string>("mavros_namespace", mavros_, "/mavros");
    std::string mode;
    if (ros::Time::isSimTime() && nh_.getParam(mavros_+"/time/timesync_mode", mode) && mode == "PASSTHROUGH")
      clock_lead_tolerance_ = .008;  // 两个4ms仿真步的消息到达顺序差；实机仍为0。

  }

  /** @brief 从 YAML 节点提取一组指定轴的 P/I/D/FF/K 参数。 */
  Candidate parseCandidate(const YAML::Node& node, const std::string& id,
                           const std::string& prefix) const {
    Candidate candidate;
    candidate.id = id;
    if (!node.IsMap() || node.size() != (id == "baseline" ? 5u : 6u))
      throw std::runtime_error("candidate requires exactly P/I/D/FF/K and id");
    for (const auto& suffix : {"P", "I", "D", "FF", "K"}) {
      const double value = node[prefix+suffix].as<double>();
      if (!std::isfinite(value) || value < 0 ||
          (std::string(suffix) == "K" && value == 0))
        throw std::runtime_error("invalid gain");
      candidate.values[prefix+suffix] = value;
    }
    return candidate;
  }

  /** @brief 加载试验配置并校验激励设置、baseline 和候选列表。 */
  void loadConfiguration(const std::string& config) {
    const auto root = YAML::LoadFile(config);
    experiment_ = root["experiment_id"].as<std::string>();
    axis_ = root["axis"].as<std::string>();

    if (experiment_.empty() || (axis_ != "roll" && axis_ != "pitch" && axis_ != "yaw"))
      throw std::runtime_error("invalid experiment/axis");

    if (root["collective_thrust"])
      ROS_WARN("collective_thrust is ignored; capture thrust from armed POSCTL before each Offboard entry");
    const auto excitation = root["excitation"];
    settle_ = excitation["settle_s"].as<double>();
    duration_ = excitation["pulse_duration_s"].as<double>();
    repetitions_ = excitation["repetitions"].as<int>();
    const auto levels = excitation["levels_deg_s"];
    if (!levels.IsSequence() || levels.size() != 2)
      throw std::runtime_error("exactly two amplitudes required");
    low_ = levels[0].as<double>();
    high_ = levels[1].as<double>();
    for (double value : {settle_, duration_, low_, high_})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("nonfinite or nonpositive excitation setting");
    if (repetitions_ < 1 || high_ < low_)
      throw std::runtime_error("invalid thrust/repetitions/amplitudes");
    const auto control = root["altitude_control"];
    const auto recovery = root["recovery"];
    auto setting = [](const YAML::Node& node, const char* key, double& value) {
      if (node && node[key]) value = node[key].as<double>();
      if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("invalid setting: ")+key);
    };
    setting(control, "kp", altitude_.kp); setting(control, "kd", altitude_.kd);
    setting(control, "ki", altitude_.ki); setting(control, "acceleration_limit", altitude_.acceleration_limit);
    setting(control, "integral_limit", altitude_.integral_limit);
    setting(control, "min_thrust", altitude_.min_thrust); setting(control, "max_thrust", altitude_.max_thrust);
    setting(recovery, "timeout_s", recovery_timeout_);
    setting(recovery, "horizontal_tolerance_m", horizontal_tolerance_);
    setting(recovery, "position_gain", recenter_.position_gain);
    setting(recovery, "velocity_gain", recenter_.velocity_gain);
    setting(recovery, "attitude_gain", recenter_.attitude_gain);
    setting(recovery, "max_tilt_deg", recenter_.max_tilt_deg);
    setting(recovery, "max_rate_rad_s", recenter_.max_rate);
    setting(recovery, "position_tolerance_m", position_tolerance_);
    setting(recovery, "speed_tolerance_m_s", speed_tolerance_);
    setting(recovery, "tilt_tolerance_deg", tilt_tolerance_deg_);
    setting(recovery, "yaw_tolerance_deg", yaw_tolerance_deg_);
    setting(recovery, "angular_speed_tolerance_rad_s", angular_speed_tolerance_);
    setting(recovery, "angular_feedback_timeout_s", angular_feedback_timeout_);
    if (altitude_.min_thrust >= altitude_.max_thrust || altitude_.max_thrust > 1 ||
        recenter_.max_tilt_deg >= 80 || tilt_tolerance_deg_ >= 80 || yaw_tolerance_deg_ >= 180 ||
        recovery_timeout_ <= settle_)
      throw std::runtime_error("inconsistent altitude/recovery limits");

    const std::string prefix = axis_ == "roll" ? "MC_ROLLRATE_" :
                               axis_ == "pitch" ? "MC_PITCHRATE_" : "MC_YAWRATE_";
    baseline_ = parseCandidate(root["baseline"], "baseline", prefix);
    const auto stage = root["search_stage"] ? root["search_stage"].as<std::string>() : "pd";
    if (stage != "pd" && stage != "manual")
      throw std::runtime_error("search_stage must be pd or manual");

    std::set<std::string> ids{"baseline"};
    for (const auto& node : root["candidates"]) {
      const auto id = node["id"].as<std::string>();
      if (id.empty() || !ids.insert(id).second)
        throw std::runtime_error("duplicate/reserved/empty candidate id");
      auto candidate = parseCandidate(node, id, prefix);
      if (!brct::matches(candidate.values.at(prefix+"K"), baseline_.values.at(prefix+"K")))
        throw std::runtime_error("K must remain fixed");
      if (stage == "pd" &&
          (candidate.values.at(prefix+"I") != 0 || candidate.values.at(prefix+"FF") != 0))
        throw std::runtime_error("pd stage requires I=FF=0");
      candidates_.push_back(candidate);
    }
    if (candidates_.empty()) throw std::runtime_error("empty candidate list");
  }

  /** @brief 创建唯一会话目录，归档配置并打开事件与时间同步日志。 */
  void prepareSession(const std::string& config, const std::string& output) {
    // 拒绝复用目录，防止节点重启后把不同会话的 trial_id 和事件静默混合。
    if (boost::filesystem::exists(output))
      throw std::runtime_error("output directory already exists; choose a new session directory");
    boost::filesystem::create_directories(output);
    boost::filesystem::copy_file(config, output+"/candidates.yaml");
    YAML::Node clock_config;
    bool simulated = false; std::string sync_mode;
    nh_.param("/use_sim_time", simulated, false);
    nh_.param<std::string>(mavros_+"/time/timesync_mode", sync_mode, "unknown");
    clock_config["use_sim_time"] = simulated;
    clock_config["mavros_timesync_mode"] = sync_mode;
    clock_config["max_sample_lead_s"] = clock_lead_tolerance_;
    std::ofstream clock_file(output+"/clock.yaml");
    clock_file << clock_config << '\n'; clock_file.flush();
    if (!clock_file) throw std::runtime_error("cannot record clock configuration");
    journal_.open(output+"/events.yaml");
    if (!journal_) throw std::runtime_error("cannot create event journal");
    sync_journal_.open(output+"/timesync.csv");
    if (!sync_journal_) throw std::runtime_error("cannot create time synchronization journal");
    sync_journal_ << "ros_s,ros_minus_px4_s,round_trip_time_ms\n";
    sync_journal_.flush();
  }

  /** @brief 注册话题、服务和 200 Hz 墙钟定时器。 */
  void setupRosInterfaces() {
    timesync_ = nh_.subscribe(mavros_+"/timesync_status", 10, &Autotune::timesync, this);
    command_ = nh_.advertise<mavros_msgs::AttitudeTarget>(mavros_+"/setpoint_raw/attitude", 1);
    imu_sub_ = nh_.subscribe(mavros_+"/imu/data", 100, &Autotune::imu, this);
    pose_sub_ = nh_.subscribe(mavros_+"/local_position/pose", 1, &Autotune::pose, this);
    velocity_sub_ = nh_.subscribe(mavros_+"/local_position/velocity_local", 1, &Autotune::velocity, this);
    events_ = nh_.advertise<Event>("/pid_experiment/event", 100);
    vehicle_ = nh_.subscribe(mavros_+"/state", 10, &Autotune::vehicle, this);
    thrust_feedback_ = nh_.subscribe(mavros_+"/setpoint_raw/target_attitude", 1,
                                    &Autotune::thrustFeedback, this);
    services_.push_back(nh_.advertiseService("/load_candidate", &Autotune::loadService, this));
    services_.push_back(nh_.advertiseService("/run_trial", &Autotune::run, this));
    services_.push_back(nh_.advertiseService("/stop_trial", &Autotune::stop, this));
    services_.push_back(nh_.advertiseService("/restore_baseline", &Autotune::baseline, this));
    services_.push_back(nh_.advertiseService("/next_candidate", &Autotune::next, this));
    services_.push_back(nh_.advertiseService("/get_experiment_status", &Autotune::status, this));
    timer_ = nh_.createWallTimer(ros::WallDuration(0.005), &Autotune::tick, this);
  }

  /**
   * @brief 发布试验事件并同步追加到 YAML 日志。
   * @param type 事件类型。
   * @param detail 可选的失败原因或补充信息。
   * @warning 日志无法写入时调用 `ros::shutdown()`，避免继续产生无法追溯的试验数据。
   */
  void event(const std::string& type, const std::string& detail = "") {
    Event e;
    e.timestamp = ros::Time::now(); e.steady_s = steady();
    e.experiment_id = experiment_; e.candidate_id = current_.id; e.event_type = type;
    e.axis = axis_; e.amplitude_deg_s = amplitude_; e.requested_parameters = dump(current_.values);
    e.readback_parameters = dump(readback_); e.trial_id = trial_; e.phase = phase_;
    e.repetition = repetition_; e.detail = detail;
    events_.publish(e);
    YAML::Node n;
    n["timestamp"] = e.timestamp.toSec(); n["steady_s"] = e.steady_s;
    n["experiment_id"] = experiment_; n["candidate_id"] = current_.id; n["event_type"] = type;
    n["axis"] = axis_; n["amplitude_deg_s"] = amplitude_; n["requested_parameters"] = current_.values;
    n["readback_parameters"] = readback_; n["trial_id"] = trial_;
    n["phase"] = phase_; n["repetition"] = repetition_; n["detail"] = detail;
    n["collective_thrust"] = thrust_;
    n["thrust_locked"] = thrust_locked_;
    n["thrust_source_timestamp"] = thrust_stamp_.toSec();
    n["origin_enu"] = std::vector<double>{origin_.x, origin_.y, origin_.z};
    n["origin_yaw_rad"] = origin_yaw_;
    n["height_error_m"] = origin_.z-position_now_.z;
    n["command_thrust"] = sent_thrust_;
    n["control_mode"] = pulse_active_ ? "body_rate_altitude" : "body_rate_recenter";
    journal_ << "---\n" << n << '\n'; journal_.flush();
    if (!journal_) { ROS_FATAL("Event journal write failed"); ros::shutdown(); }
  }

  /** @brief 将 ROS 时间、ROS-PX4 偏移和往返时间逐样本写入 CSV。 */
  void timesync(const mavros_msgs::TimesyncStatus::ConstPtr& msg) {
    sync_journal_ << std::setprecision(17) << msg->header.stamp.toSec() << ','
                  << msg->estimated_offset_ns*1e-9 << ',' << msg->round_trip_time_ms << '\n';
    sync_journal_.flush();
    if (!sync_journal_) { ROS_FATAL("Time synchronization journal write failed"); ros::shutdown(); }
  }

  /**
   * @brief 激励时发布单轴角速度和高度闭环推力，其余 Offboard 阶段以角速度闭环回中。
   * @param frd_rate FRD 机体系目标角速度，单位为 rad/s；默认发布零角速度。
   */
  void publish(double frd_rate = 0) {
    // 没有可信的交接推力时不提供 Offboard 指令流，避免以旧值或零推力接管。
    if (!thrustReady(1.5)) {
      ROS_INFO_THROTTLE(5.0, "Pre-Offboard thrust: %s", thrustWaitReason().c_str());
      return;
    }
    if (inOffboard() && (!origin_locked_ || !feedbackFresh())) return;
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now(); msg.header.frame_id = "base_link";
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.orientation.w = 1; msg.thrust = thrust_;
    if (inOffboard() && origin_locked_) {
      const ros::Time now = ros::Time::now();
      const double dt = std::max(0.0, (now-control_stamp_).toSec());
      control_stamp_ = now;
      msg.thrust = altitude_.update(thrust_, origin_.z-position_now_.z,
        velocity_now_.z, std::max(cos_tilt_, std::sqrt(.5)), dt);
    }
    sent_thrust_ = msg.thrust;
    if (inOffboard() && !pulse_active_) {
      const Eigen::Vector3d error(origin_.x-position_now_.x, origin_.y-position_now_.y, origin_.z-position_now_.z);
      const Eigen::Vector3d velocity(velocity_now_.x, velocity_now_.y, velocity_now_.z);
      const auto rates = recenter_.rates(attitude_, error, velocity, origin_yaw_);
      msg.body_rate.x = rates.x(); msg.body_rate.y = rates.y(); msg.body_rate.z = rates.z();
    } else {
      // MAVROS 输入 FLU；配置和事件采用 FRD。
      if (axis_ == "roll") msg.body_rate.x = frd_rate;
      if (axis_ == "pitch") msg.body_rate.y = -frd_rate;
      if (axis_ == "yaw") msg.body_rate.z = -frd_rate;
    }
    command_.publish(msg);
  }

  /** @brief 中止当前脉冲及整套自动序列；已开始的参数事务仍会返回结果。 */
  void abort(const std::string& why) {
    const bool running = state_ == "RUNNING";
    const bool automatic = sequence_active_;
    pulse_active_ = false;
    stage_started_ = steady();
    sequence_active_ = false;
    if (!running && !automatic) return;
    if (state_ != "PARAMETER_LOADING") state_ = "ABORTED";
    publish();
    if (running) event("TRIAL_ABORTED", why);
    if (automatic) event("SEQUENCE_ABORTED", why);
  }

  /** @brief FCU 必须已连接、处于 Offboard，且状态消息未超过 2 s。 */
  bool inOffboard() const { return offboard_ && steady()-state_received_ < state_timeout_; }

  bool freshThrust(double max_age = .5) const {
    return thrust_received_ > 0 && steady()-thrust_received_ <= max_age;
  }

  bool thrustReady(double max_age = .5) const {
    return inOffboard() ? thrust_locked_ :
      (!offboard_ && position_armed_ && steady()-state_received_ < state_timeout_ && freshThrust(max_age) && feedbackFresh());
  }

  bool feedbackFresh() const {
    return angularFeedbackFresh() && pose_received_ > 0 && velocity_received_ > 0 &&
      steady()-pose_received_ < feedback_timeout_ && steady()-velocity_received_ < feedback_timeout_;
  }

  void pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const auto& p = msg->pose.position; const auto& q = msg->pose.orientation;
    const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < .5 || norm > 1.5 || age < -clock_lead_tolerance_ || age > feedback_timeout_ ||
        msg->header.stamp <= pose_stamp_) return;
    position_now_ = p;
    attitude_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    cos_tilt_ = brct::bound(1-2*(q.x*q.x+q.y*q.y)/norm, -1, 1);
    yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y), norm-2*(q.y*q.y+q.z*q.z));
    pose_received_ = steady(); pose_stamp_ = msg->header.stamp;
  }

  void velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const auto& v = msg->twist.linear;
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
        age < -clock_lead_tolerance_ || age > feedback_timeout_ || msg->header.stamp <= velocity_stamp_) return;
    velocity_now_ = v; velocity_received_ = steady(); velocity_stamp_ = msg->header.stamp;
  }

  bool angularFeedbackFresh() const {
    const double age = (ros::Time::now()-imu_stamp_).toSec();
    // 样本年龄按ROS/飞控时钟判断；独立墙钟看门狗识别通信失联。
    // SITL参数落盘可暂停物理时钟，不能把103ms墙钟停顿当成103ms飞行状态过期。
    return imu_received_ > 0 && brct::freshSample(age, steady()-imu_received_,
                                                angular_feedback_timeout_, feedback_timeout_, clock_lead_tolerance_);
  }

  void imu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& w = msg->angular_velocity;
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    const double speed = std::hypot(std::hypot(w.x, w.y), w.z);
    if (!std::isfinite(speed) || msg->angular_velocity_covariance[0] == -1 ||
        age < -clock_lead_tolerance_ || age > angular_feedback_timeout_ || msg->header.stamp <= imu_stamp_) {
      settling_.reset(); return;
    }
    angular_speed_ = speed; imu_stamp_ = msg->header.stamp; imu_received_ = steady();
    // 每条反馈都检查，避免定时器之间的振荡峰值被遗漏。
    settling_.observe(imu_stamp_.toSec(), state_ == "RUNNING" && !pulse_active_ && atOrigin(),
                      angular_feedback_timeout_);
  }

  bool atOrigin() const {
    const double yaw_error = std::atan2(std::sin(yaw_-origin_yaw_), std::cos(yaw_-origin_yaw_));
    return feedbackFresh() && angular_speed_ <= angular_speed_tolerance_ && std::hypot(origin_.x-position_now_.x, origin_.y-position_now_.y) <= horizontal_tolerance_ &&
      std::abs(origin_.z-position_now_.z) <= position_tolerance_ &&
      std::hypot(std::hypot(velocity_now_.x, velocity_now_.y), velocity_now_.z) <= speed_tolerance_ &&
      cos_tilt_ >= std::cos(tilt_tolerance_deg_*std::acos(-1.0)/180) &&
      std::abs(yaw_error) <= yaw_tolerance_deg_*std::acos(-1.0)/180;
  }

  void safetyExit(const std::string& why) {
    if (safety_exit_) return;
    safety_exit_ = true;
    abort(why);
    waiting_for_exit_ = true;
    state_ = "SAFETY_EXIT";
    event("SAFETY_ABORT", why);
    ROS_ERROR("Trial protection: %s; requesting POSCTL", why.c_str());
  }

  std::string thrustWaitReason() const {
    if (state_received_ == 0 || steady()-state_received_ >= state_timeout_)
      return "waiting for fresh FCU state";
    if (offboard_)
      return "no pre-entry thrust locked; return to Position hover to acquire it";
    if (!position_armed_)
      return "waiting for armed POSCTL; take off and establish Position hover first";
    if (!feedbackFresh()) return "waiting for fresh local position, velocity and IMU";
    if (thrust_received_ == 0)
      return "waiting for valid FCU target_attitude thrust in POSCTL";
    return "FCU target_attitude thrust feedback is stale";
  }

  /** @brief 仅在已解锁 Position 模式接收新鲜的飞控推力目标，拒绝回传旧包。 */
  void thrustFeedback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (offboard_ || !position_armed_ || steady()-state_received_ >= state_timeout_) return;
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (msg->header.stamp <= position_since_ || msg->header.stamp <= thrust_stamp_ ||
        age < -clock_lead_tolerance_ || age > thrust_timeout_ ||
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) ||
        !std::isfinite(msg->thrust) || msg->thrust <= 0 || msg->thrust > 1) return;
    const bool was_ready = thrustReady();
    thrust_ = msg->thrust;
    thrust_received_ = steady();
    thrust_stamp_ = msg->header.stamp;
    if (!was_ready && thrustReady()) {
      event("HOVER_THRUST_READY", "pre-entry FCU thrust acquired; tracking in POSCTL");
      ROS_INFO("Pre-Offboard thrust ready: %.6f; tracking FCU thrust and streaming setpoints in POSCTL. "
               "Switch only after stable hover; thrust will then be locked.", thrust_);
    }
  }

  /** @brief 更新 FCU 模式与连接状态；运行期间失去 Offboard 会立即中止。 */
  void vehicle(const mavros_msgs::State::ConstPtr& msg) {
    bool next = msg->connected && msg->armed && msg->mode == "OFFBOARD";
    const bool entering = next && !offboard_;
    const bool position = msg->connected && msg->armed && msg->mode == "POSCTL";
    const bool state_fresh = steady()-state_received_ < state_timeout_;
    if (next && !offboard_) {
      safety_exit_ = false;
      // 模式心跳可滞后 1 s；切入角速度 Offboard 后推力反馈已停止，交接与预发送共用 1.5 s 宽限。
      thrust_locked_ = position_armed_ && state_fresh && freshThrust(1.5) && feedbackFresh();
      origin_locked_ = thrust_locked_;
      if (origin_locked_) {
        origin_ = position_now_; origin_yaw_ = yaw_;
        altitude_.integral = 0;
        control_stamp_ = ros::Time::now();
        event("ORIGIN_CAPTURED", "ENU origin and yaw; altitude hold and body-rate recenter between pulse pairs");
      }
      event(thrust_locked_ ? "HOVER_THRUST_CAPTURED" : "HOVER_THRUST_UNAVAILABLE",
            "source=FCU target_attitude in armed POSCTL; thrust=" + std::to_string(thrust_));
      if (!thrust_locked_) ROS_ERROR("Offboard entered without fresh pre-entry hover thrust; trials disabled");
      else ROS_INFO("Offboard entered: pre-acquired thrust locked at %.6f", thrust_);
    }
    if (!next) {
      thrust_locked_ = false;
      origin_locked_ = false;
      if (!position || !position_armed_ || !state_fresh) {
        thrust_received_ = 0;
        thrust_stamp_ = ros::Time();
        position_since_ = ros::Time::now();
      }
    }
    position_armed_ = position;
    state_received_ = steady();
    if (next != offboard_) { offboard_ = next; event(next ? "OFFBOARD_ENTERED" : "OFFBOARD_EXITED"); }
    if (!next) abort("manual exit or FCU disconnected");
    if (waiting_for_exit_ && !next) {
      waiting_for_exit_ = false;
      state_ = msg->connected && msg->mode == "POSCTL" ? (safety_exit_ ? "ABORTED" : "COMPLETE") : "EXIT_CANCELLED";
      event(state_ == "ABORTED" ? "SAFETY_EXIT_CONFIRMED" :
            state_ == "COMPLETE" ? "SEQUENCE_COMPLETE" : "EXIT_CANCELLED", msg->mode);
    }
    // 只在首次连接且尚未选择候选时预加载；失败由操作者处理，不循环重写参数。
    if (msg->connected && msg->mode != "OFFBOARD" && current_.id.empty()) {
      std::string message;
      load(candidates_.front(), 0, message);
      ROS_INFO("Preparing first candidate %s before Offboard: %s", current_.id.c_str(), message.c_str());
    }
    if (entering && !sequence_done_) {
      if (!thrust_locked_ || !origin_locked_) {
        event("AUTO_RUN_REJECTED", "no fresh pre-entry hover thrust");
        safetyExit("Offboard handover rejected; return to Position for fresh hover thrust");
      } else {
        sequence_active_ = true;
        sequence_restart_ = true;
        high_level_ = false;
        event("SEQUENCE_START", "all candidates: low then high");
      }
    }
  }

  /**
   * @brief 在后台线程写入参数，强制刷新 MAVROS 缓存后逐项验证 FCU 读回值。
   * @param values 待写入的 PX4 参数名和值。
   * @return 写入结果、实际读回参数和失败原因。
   * @note 失败前已成功写入的参数不会自动回滚。
   */
  ApplyResult apply(Parameters values) {
    // 后台线程不持有状态机写权限，阻塞的 MAVLink 服务不会阻塞 200 Hz setpoint 发布。
    ApplyResult out;
    try {
      auto set = nh_.serviceClient<mavros_msgs::ParamSet>(mavros_ + "/param/set");
      auto pull = nh_.serviceClient<mavros_msgs::ParamPull>(mavros_ + "/param/pull");
      auto get = nh_.serviceClient<mavros_msgs::ParamGet>(mavros_ + "/param/get");
      // connected 只代表心跳已建立，首次参数同步可能尚未完成。
      // MAVROS ParamSet 依赖本地参数表中的类型，必须先拉取，不能直接写。
      mavros_msgs::ParamPull initial;
      initial.request.force_pull = true;
      if (!pull.call(initial) || !initial.response.success)
        throw std::runtime_error("parameter table pull before write failed");
      // 完整检查候选的所有参数是否已知，避免缺项时只写入半组。
      for (const auto& p : values) {
        mavros_msgs::ParamGet s;
        s.request.param_id = p.first;
        if (!get.call(s) || !s.response.success)
          throw std::runtime_error("parameter unavailable before write: " + p.first);
      }
      for (const auto& p : values) {
        mavros_msgs::ParamSet s; s.request.param_id = p.first; s.request.value.real = p.second;
        if (!set.call(s) || !s.response.success) throw std::runtime_error("write failed: " + p.first);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1800));
      // ParamGet 可能只返回 MAVROS 缓存；先强制从 FCU 全量拉取，再核对实际值。
      mavros_msgs::ParamPull refresh; refresh.request.force_pull = true;
      if (!pull.call(refresh) || !refresh.response.success) throw std::runtime_error("fresh parameter pull failed");
      bool matched = true;
      for (const auto& p : values) {
        mavros_msgs::ParamGet s; s.request.param_id = p.first;
        if (!get.call(s) || !s.response.success) throw std::runtime_error("read failed: " + p.first);
        out.readback[p.first] = s.response.value.real;
        matched = brct::matches(out.readback[p.first], p.second) && matched;
      }
      if (!matched) throw std::runtime_error("parameter readback mismatch");
      out.ok = true;
    } catch (const std::exception& e) { out.detail = e.what(); }
    return out;
  }

  /** @brief 接受候选并启动异步参数写入；完成前状态保持 PARAMETER_LOADING。 */
  bool load(const Candidate& candidate, int index, std::string& message) {
    if (state_ == "RUNNING" || state_ == "PARAMETER_LOADING") { message = "busy; stop trial or await parameter operation"; return false; }
    current_ = candidate; candidate_index_ = index; confirmed_ = false; readback_.clear();
    error_.clear(); amplitude_ = 0; phase_ = repetition_ = -1;
    state_ = "PARAMETER_LOADING"; event("PARAMETER_APPLY_START");
    pending_ = std::async(std::launch::async, &Autotune::apply, this, candidate.values);
    message = "accepted; wait for READY / PARAMETER_CONFIRMED before run"; return true;
  }

  /**
   * @brief 按 ID 请求加载配置中的候选参数。
   * @note C++ 回调返回值始终表示服务传输已处理；业务结果写入 `res.success`。
   */
  bool loadService(Command::Request& req, Command::Response& res) {
    if (sequence_active_ || waiting_for_exit_) {
      res.success = false; res.message = "automatic sequence busy"; return true;
    }
    for (size_t i = 0; i < candidates_.size(); ++i) if (candidates_[i].id == req.value) {
      res.success = load(candidates_[i], i, res.message); return true;
    }
    res.success = false; res.message = "unknown candidate"; return true;
  }

  /**
   * @brief 以 `low` 或 `high` 幅值启动一次已确认参数的激励试验。
   * @note 仅在参数已读回确认且 FCU Offboard 状态新鲜时接受请求。
   */
  bool run(Command::Request& req, Command::Response& res) {
    if (sequence_active_ || waiting_for_exit_) {
      res.success = false; res.message = "automatic sequence busy"; return true;
    }
    return startTrial(req.value, res);
  }

  bool startTrial(const std::string& level, Command::Response& res) {
    res.success = false;
    if (level != "low" && level != "high") res.message = "expected low or high";
    else if (state_ == "PARAMETER_LOADING") res.message = "candidate parameters are still loading; wait for READY";
    else if (state_ == "RUNNING") res.message = "trial already running";
    else if (!confirmed_) res.message = "candidate unconfirmed: " + (error_.empty() ? std::string("load a candidate first") : error_);
    else if (!inOffboard()) res.message = "not in Offboard (or vehicle state stale)";
    else if (!thrust_locked_) res.message = "no fresh hover thrust captured before Offboard";
    else if (!origin_locked_ || !feedbackFresh() || safety_exit_) res.message = "origin/feedback unavailable or protection active";
    else {
      amplitude_ = level == "low" ? low_ : high_;
      ++trial_; phase_ = repetition_ = 0; started_ = steady(); state_ = "RUNNING";
      pulse_index_ = 0; pulse_active_ = false; stage_started_ = steady(); settling_.reset();
      event("TRIAL_START"); res.success = true; res.message = "trial started";
      publish(); event("RECOVERY_START", "initial origin settling");
    }
    return true;
  }

  /** @brief 人工中止当前激励并发布零角速度；不会取消正在进行的参数操作。 */
  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    abort("operator stop"); publish(); res.success = true;
    res.message = "trial stopped; recentering while in Offboard; parameter operation is not cancelled"; return true;
  }

  /** @brief 异步写回配置文件中的 baseline；成功响应仅表示请求已接受。 */
  bool baseline(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (sequence_active_ || waiting_for_exit_) {
      res.success = false; res.message = "automatic sequence busy"; return true;
    }
    res.success = load(baseline_, -1, res.message); return true;
  }

  /** @brief 按配置顺序加载下一候选，到达末尾后不循环。 */
  bool next(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (sequence_active_ || waiting_for_exit_) {
      res.success = false; res.message = "automatic sequence busy"; return true;
    }
    int i = candidate_index_ + 1;
    if (i >= static_cast<int>(candidates_.size())) { res.success = false; res.message = "end of candidate list"; }
    else res.success = load(candidates_[i], i, res.message);
    return true;
  }

  /** @brief 返回状态、候选、参数确认、Offboard 新鲜度、错误和读回值。 */
  bool status(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    res.success = true;
    res.message = "state=" + state_ + " candidate=" + current_.id + " confirmed=" + (confirmed_ ? "true" : "false")
      + " offboard=" + (inOffboard() ? "true" : "false") + " error=" + error_ + " readback=" + dump(readback_);
    res.message += " thrust=" + std::to_string(thrust_) +
      " thrust_ready=" + (thrustReady() ? "true" : "false") +
      " thrust_locked=" + (thrust_locked_ ? "true" : "false");
    res.message += std::string(" sequence_active=") + (sequence_active_ ? "true" : "false") +
      " level=" + (high_level_ ? "high" : "low");
    if (!thrustReady()) res.message += " thrust_wait_reason=" + thrustWaitReason();
    res.message += " origin_locked=" + std::string(origin_locked_ ? "true" : "false") +
      " control=" + (pulse_active_ ? "body_rate_altitude" : "body_rate_recenter") +
      " height_error_m=" + std::to_string(origin_.z-position_now_.z) +
      " angular_speed_rad_s=" + std::to_string(angular_speed_);
    return true;
  }

  /** @brief 参数确认后执行 low/high，再推进下一候选；完成后异步退出模式。 */
  void advanceSequence() {
    if (!sequence_active_ || state_ == "PARAMETER_LOADING" || state_ == "RUNNING") return;
    std::string message;
    if (sequence_restart_) {
      sequence_restart_ = false;
      if (candidate_index_ != 0 || !confirmed_) {
        if (!load(candidates_.front(), 0, message)) abort(message);
        return;
      }
    } else if (state_ == "FINISHED") {
      if (!high_level_) high_level_ = true;
      else if (candidate_index_+1 < static_cast<int>(candidates_.size())) {
        high_level_ = false;
        const int index = candidate_index_+1;
        if (!load(candidates_[index], index, message)) abort(message);
        return;
      } else {
        sequence_active_ = false;
        sequence_done_ = true;
        waiting_for_exit_ = true;
        state_ = "WAIT_POSCTL";
        event("SEQUENCE_EXCITATION_FINISHED", "requesting POSCTL");
        return;
      }
    }
    Command::Response response;
    startTrial(high_level_ ? "high" : "low", response);
    if (!response.success) abort(response.message);
  }

  /** @brief 后台请求 POSCTL，最多每 0.5 s 重试；成功以 state 回传为准。 */
  void retryExit() {
    if (exit_request_.valid()) {
      if (exit_request_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
      if (!exit_request_.get()) ROS_WARN("POSCTL request failed; waiting to retry");
    }
    if (!waiting_for_exit_ || !inOffboard() || steady()-last_exit_request_ < 0.5) return;
    last_exit_request_ = steady();
    exit_request_ = std::async(std::launch::async, [this]() {
      auto client = nh_.serviceClient<mavros_msgs::SetMode>(mavros_+"/set_mode");
      mavros_msgs::SetMode request;
      request.request.custom_mode = "POSCTL";
      return client.call(request) && request.response.mode_sent;
    });
  }

  /** @brief 每个正负脉冲后闭环回中；只在真实运动状态稳定后继续。 */
  void beginPulse(double now) {
    repetition_ = pulse_index_/4; phase_ = (pulse_index_%4)*2+1;
    pulse_active_ = true; stage_started_ = now; settling_.reset();
    const double sign = (pulse_index_%4 == 0 || pulse_index_%4 == 3) ? 1 : -1;
    publish(sign*amplitude_*std::acos(-1.0)/180);
    event(sign > 0 ? "PULSE_POSITIVE" : "PULSE_NEGATIVE");
  }

  void advancePulse() {
    if (state_ != "RUNNING") return;
    const double now = steady();
    if (pulse_active_) {
      if (now-stage_started_ < duration_) return;
      if (now-stage_started_ > duration_+.1) { safetyExit("pulse scheduler overrun"); return; }
      event("PULSE_FINISHED");
      pulse_active_ = false; ++pulse_index_; ++phase_;
      stage_started_ = now; settling_.reset();
      publish(); event("RECOVERY_START", "pulse complete; attitude, height, velocity and measured body-rate settling");
      return;
    }
    if (now-stage_started_ > recovery_timeout_) { safetyExit("recenter timeout"); return; }
    if (!atOrigin()) { settling_.reset(); return; }
    if (!settling_.ready(settle_)) return;
    event("RECOVERY_FINISHED", "height, speed, tilt, yaw and measured body rates stable on continuous fresh feedback");
    if (pulse_index_ == 4*repetitions_) {
      state_ = "FINISHED"; event("TRIAL_FINISHED"); return;
    }
    beginPulse(now);
  }

  /**
   * @brief 轮询异步参数结果，推进独立脉冲/回位阶段。
   *
   * 墙钟定时器周期为 5 ms，不受 ROS 仿真时钟暂停或跳变影响。
   * 调度延迟若跨过完整阶段则中止，不补发已经错过的虚构脉冲。
   * 每次回调末尾均发布当前指令，使 MAVLink setpoint 流持续存在。
   */
  void tick(const ros::WallTimerEvent&) {
    if (state_ == "PARAMETER_LOADING" && pending_.valid() &&
        pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = pending_.get(); readback_ = result.readback; confirmed_ = result.ok;
      error_ = result.detail;
      if (!waiting_for_exit_) state_ = result.ok ? "READY" : "IDLE";
      event(result.ok ? "PARAMETER_CONFIRMED" : "PARAMETER_APPLY_FAILED", error_);
      if (result.ok) ROS_INFO("Candidate %s READY: all parameters confirmed from FCU", current_.id.c_str());
      else ROS_ERROR("Candidate %s parameter loading failed: %s", current_.id.c_str(), error_.c_str());
      if (result.ok && current_.id == "baseline") event("BASELINE_RESTORED");
      if (!result.ok) abort("parameter apply failed: " + error_);
    }
    if ((sequence_active_ || state_ == "RUNNING") && !inOffboard()) abort("Offboard state unavailable or stale");
    advanceSequence();
    retryExit();
    advancePulse();
    const double sign = (pulse_index_%4 == 0 || pulse_index_%4 == 3) ? 1 : -1;
    const double rate = pulse_active_ ? sign*amplitude_*std::acos(-1.0)/180 : 0;
    publish(rate);
  }

 public:
  /**
   * @brief 依次加载参数与配置、准备会话文件并建立 ROS 接口。
   * @throws std::exception 参数缺失、配置非法、输出目录已存在或日志无法创建。
   */
  Autotune() {
    if (!ros::Time::waitForValid(ros::WallDuration(30)))
      throw std::runtime_error("No valid ROS clock after 30 s; start Gazebo /clock before the experiment");
    std::string config, output;
    loadStartupParameters(config, output);
    loadConfiguration(config);
    prepareSession(config, output);
    setupRosInterfaces();
    event("SESSION_START");
  }

  /** @brief 销毁前等待已启动的参数事务结束，避免后台线程继续访问本对象。 */
  ~Autotune() {
    if (pending_.valid()) pending_.wait();
    if (exit_request_.valid()) exit_request_.wait();
  }
};

/* ============================ 节点入口 ============================ */

int main(int argc, char** argv) {
  ros::init(argc, argv, "brct_autotune_fsm");
  try { Autotune node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
