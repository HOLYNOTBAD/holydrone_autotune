/**
 * @file brct_autotune_fsm.cpp
 * @brief 管理 PX4 角速度环候选参数的加载、读回验证和人工触发激励。
 *
 * @details
 * 节点通过 MAVROS 参数服务异步写入候选参数，并持续向
 * `setpoint_raw/attitude` 发布角速度与固定推力。试验仅在 FCU 状态新鲜且处于
 * Offboard 时启动；退出 Offboard、状态超时或调度跨越完整激励阶段时立即清零角速度并中止。
 * 节点不会解锁、切换模式、选择最优参数或在退出时自动恢复 baseline。
 *
 * ROS 接口：
 * - 订阅 `<mavros_namespace>/state` 和 `<mavros_namespace>/timesync_status`；
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
#include <std_srvs/Trigger.h>
#include <holydrone_autotune/ExperimentCommand.h>
#include <holydrone_autotune/ExperimentEvent.h>
#include <holydrone_autotune/brct_trial.hpp>

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
  ros::Subscriber vehicle_, timesync_;
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
  double thrust_, settle_, duration_, low_, high_, amplitude_ = 0, started_ = 0;
  /** @brief 最近 FCU 状态的单调接收时间及允许陈旧时间，单位为 s。 */
  double state_received_ = 0, state_timeout_ = 2.0;
  /** @brief Offboard 状态缓存，以及当前参数是否通过 FCU 读回确认。 */
  bool offboard_ = false, confirmed_ = false;
  /** @brief 当前候选、激励阶段和重复轮次索引；-1 表示尚未进入。 */
  int repetitions_, candidate_index_ = -1, phase_ = -1, repetition_ = -1;
  /** @brief 本次节点会话内单调递增的试验编号。 */
  uint32_t trial_ = 0;

  /** @brief 读取节点启动所需的私有 ROS 参数。 */
  void loadStartupParameters(std::string& config, std::string& output) {
    if (!private_.getParam("config", config) || !private_.getParam("output", output))
      throw std::runtime_error("config and unique output directory required");
    private_.param<std::string>("mavros_namespace", mavros_, "/mavros");
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

    thrust_ = root["collective_thrust"].as<double>();
    const auto excitation = root["excitation"];
    settle_ = excitation["settle_s"].as<double>();
    duration_ = excitation["pulse_duration_s"].as<double>();
    repetitions_ = excitation["repetitions"].as<int>();
    const auto levels = excitation["levels_deg_s"];
    if (!levels.IsSequence() || levels.size() != 2)
      throw std::runtime_error("exactly two amplitudes required");
    low_ = levels[0].as<double>();
    high_ = levels[1].as<double>();
    for (double value : {thrust_, settle_, duration_, low_, high_})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("nonfinite or nonpositive excitation setting");
    if (thrust_ > 1 || repetitions_ < 1 || high_ < low_)
      throw std::runtime_error("invalid thrust/repetitions/amplitudes");

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
    events_ = nh_.advertise<Event>("/pid_experiment/event", 100);
    vehicle_ = nh_.subscribe(mavros_+"/state", 10, &Autotune::vehicle, this);
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
   * @brief 发布固定推力和单轴角速度指令。
   * @param frd_rate FRD 机体系目标角速度，单位为 rad/s；默认发布零角速度。
   */
  void publish(double frd_rate = 0) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now(); msg.header.frame_id = "base_link";
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.orientation.w = 1; msg.thrust = thrust_;
    // MAVROS 接口采用 FLU；配置与事件采用 FRD，因此 pitch/yaw 在交给 MAVROS 前取反。
    if (axis_ == "roll") msg.body_rate.x = frd_rate;
    if (axis_ == "pitch") msg.body_rate.y = -frd_rate;
    if (axis_ == "yaw") msg.body_rate.z = -frd_rate;
    command_.publish(msg);
  }

  /** @brief 仅在运行态中止试验，立即发布零角速度并记录原因。 */
  void abort(const std::string& why) {
    if (state_ != "RUNNING") return;
    state_ = "ABORTED"; publish(); event("TRIAL_ABORTED", why);
  }

  /** @brief FCU 必须已连接、处于 Offboard，且状态消息未超过 2 s。 */
  bool inOffboard() const { return offboard_ && steady()-state_received_ < state_timeout_; }

  /** @brief 更新 FCU 模式与连接状态；运行期间失去 Offboard 会立即中止。 */
  void vehicle(const mavros_msgs::State::ConstPtr& msg) {
    bool next = msg->connected && msg->mode == "OFFBOARD";
    state_received_ = steady();
    if (next != offboard_) { offboard_ = next; event(next ? "OFFBOARD_ENTERED" : "OFFBOARD_EXITED"); }
    if (!next) abort("manual exit or FCU disconnected");
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
    res.success = false;
    if (req.value != "low" && req.value != "high") res.message = "expected low or high";
    else if (!confirmed_ || state_ == "RUNNING" || state_ == "PARAMETER_LOADING") res.message = "candidate unconfirmed or busy";
    else if (!inOffboard()) res.message = "not in Offboard (or vehicle state stale)";
    else {
      amplitude_ = req.value == "low" ? low_ : high_;
      ++trial_; phase_ = repetition_ = -1; started_ = steady(); state_ = "RUNNING";
      event("TRIAL_START"); res.success = true; res.message = "trial started";
    }
    return true;
  }

  /** @brief 人工中止当前激励并发布零角速度；不会取消正在进行的参数操作。 */
  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    abort("operator stop"); publish(); res.success = true;
    res.message = "zero rates; parameter operation, if any, is not cancelled"; return true;
  }

  /** @brief 异步写回配置文件中的 baseline；成功响应仅表示请求已接受。 */
  bool baseline(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    res.success = load(baseline_, -1, res.message); return true;
  }

  /** @brief 按配置顺序加载下一候选，到达末尾后不循环。 */
  bool next(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
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
    return true;
  }

  /**
   * @brief 轮询异步参数结果并按单调时间推进七阶段对称激励。
   *
   * 墙钟定时器周期为 5 ms，不受 ROS 仿真时钟暂停或跳变影响。
   * 调度延迟若跨过完整阶段则中止，不补发已经错过的虚构脉冲。
   * 每次回调末尾均发布当前指令，使 MAVLink setpoint 流持续存在。
   */
  void tick(const ros::WallTimerEvent&) {
    if (state_ == "PARAMETER_LOADING" && pending_.valid() &&
        pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = pending_.get(); readback_ = result.readback; confirmed_ = result.ok;
      error_ = result.detail; state_ = result.ok ? "READY" : "IDLE";
      event(result.ok ? "PARAMETER_CONFIRMED" : "PARAMETER_APPLY_FAILED", error_);
      if (result.ok && current_.id == "baseline") event("BASELINE_RESTORED");
    }
    if (state_ == "RUNNING" && !inOffboard()) abort("Offboard state unavailable or stale");
    double rate = 0;
    if (state_ == "RUNNING") {
      const auto p = brct::pulse(steady()-started_, settle_, duration_, repetitions_);
      if (p.finished) {
        if (repetition_ != repetitions_-1 || phase_ != 6) abort("scheduler skipped excitation phase");
        else { state_ = "FINISHED"; publish(); event("TRIAL_FINISHED"); }
      }
      else {
        // 调度间隙后不追赶补发；跨越完整阶段会破坏试验时序，必须中止。
        const int previous = repetition_ < 0 ? -1 : repetition_*7+phase_;
        if (p.repetition*7+p.phase > previous+1) abort("scheduler skipped excitation phase");
        else {
          rate = p.sign*amplitude_*std::acos(-1.0)/180.0;
          if (p.phase != phase_ || p.repetition != repetition_) {
            phase_ = p.phase; repetition_ = p.repetition;
            publish(rate);
            event(p.sign > 0 ? "PULSE_POSITIVE" : p.sign < 0 ? "PULSE_NEGATIVE" : "PULSE_ZERO");
          }
        }
      }
    }
    publish(rate);
  }

 public:
  /**
   * @brief 依次加载参数与配置、准备会话文件并建立 ROS 接口。
   * @throws std::exception 参数缺失、配置非法、输出目录已存在或日志无法创建。
   */
  Autotune() {
    std::string config, output;
    loadStartupParameters(config, output);
    loadConfiguration(config);
    prepareSession(config, output);
    setupRosInterfaces();
    event("SESSION_START");
  }

  /** @brief 销毁前等待已启动的参数事务结束，避免后台线程继续访问本对象。 */
  ~Autotune() { if (pending_.valid()) pending_.wait(); }
};

/* ============================ 节点入口 ============================ */

int main(int argc, char** argv) {
  ros::init(argc, argv, "brct_autotune_fsm");
  try { Autotune node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
