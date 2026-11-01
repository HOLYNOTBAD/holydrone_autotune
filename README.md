# PX4 rate 环在线候选试验（ROS 1）

入口 `brct_autotune_fsm.cpp`，与原有 `identification_fsm` 独立。按确认的接口选择，使用 ROS 1 Noetic + MAVROS 的 MAVLink body-rate 控制，不发布 ROS 2 `/fmu/*`。不解锁、不切模式、不自动恢复、不自动选优；一次启动可测试整个候选列表。

## 接口与数据约定

- `/mavros/setpoint_raw/attitude`：200 Hz，`IGNORE_ATTITUDE`，角速度和推力有效。MAVLink setpoint 流同时作为 Offboard 存活信号，无需单独 ROS 2 heartbeat。
- 配置、事件、报告均使用 FRD。MAVROS 消息使用 FLU，因此 pitch/yaw 发布前取反；MAVROS 再转换到 PX4 FRD。推力为归一化正标量 `collective_thrust`，对应 PX4 负 Z 推力。
- 必须核对 MAVROS `setpoint_raw/thrust_scaling: 1.0` 已配置，并只运行一个 setpoint 发布节点。不要同时启动原有激励节点或 policy FSM。
- `/mavros/state` 的 `connected && mode == OFFBOARD` 允许运行；收到退出消息即清零并 ABORT。状态超过 2 秒未更新，也视为无法确认 Offboard 并中止。
- 未运行期间仍发送零角速度和配置的固定推力。零角速度不是位置悬停，也不会自动保持高度或恢复水平。
- 参数通过 `/mavros/param/set` 写入，等待 1.8 秒，再 `param/pull(force_pull=true)` 强制刷新，逐项 `param/get` 验证。全量刷新可能再耗时数秒。容差为 `1e-7 + 1e-5*abs(target)`。
- 参数服务运行在后台，200 Hz 发布不等待 MAVLink 事务。加载未完成时不能运行或再次加载。写入部分成功后失败会保留实际已写入的参数，明确标记失败且禁止激励；不会自动回滚。

官方接口依据：[PX4 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)、[MAVROS 坐标变换实现](https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp)、[MAVROS 参数缓存实现](https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/param.cpp)。

## 构建、启动

```bash
cd /home/holy/QUAD_RL/deployment
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
# MAVROS 需另行启动并连接到当前飞控
roslaunch holydrone_autotune brct_autotune.launch \
  output:=/data/roll_p_coarse_01_flight01
```

`output` 必须为全新目录；重启使用另一目录。节点保存原始 YAML、逐事件刷盘的 `events.yaml` 和用于对齐的 `timesync.csv`。不启动 record，不录制 ROS bag；所有飞行遥测均使用 PX4 ULog（`.ulg`）。

`config:=/absolute/path/candidates.yaml` 可替换配置。每组必须且只能包含所选轴的 P/I/D/FF/K。默认 `search_stage: pd` 强制所有候选 I=FF=0，K 固定为 baseline K；后续 I/FF 搜索使用 `manual`，K 仍固定。baseline 可保留原始 I/FF，以便恢复实际基线。K0 应由操作者填入，程序不会读取 Autotune 结果或推断 K0。重复/保留 ID、NaN、缺项、空候选会导致启动失败。

```bash
rosrun holydrone_autotune brct_autotune_command.py load p_1p00
rosrun holydrone_autotune brct_autotune_command.py status
# 等待 READY，操作者遥控进入 Offboard
rosrun holydrone_autotune brct_autotune_command.py run low
# 等待 FINISHED 后再人工触发
rosrun holydrone_autotune brct_autotune_command.py run high
rosrun holydrone_autotune brct_autotune_command.py stop
# 人工退出 Offboard，再选下一组；next 从当前列表位置向后，不循环
rosrun holydrone_autotune brct_autotune_command.py next
rosrun holydrone_autotune brct_autotune_command.py baseline
```

也可直接调用服务：

```bash
rosservice call /load_candidate "value: 'p_1p00'"
rosservice call /run_trial "value: 'low'"
rosservice call /stop_trial
rosservice call /next_candidate
rosservice call /restore_baseline
rosservice call /get_experiment_status
```

load/next/baseline 返回成功仅表示接受异步加载，必须等 `PARAMETER_CONFIRMED`/READY；baseline 另发 `BASELINE_RESTORED`。运行中禁止加载、恢复和第二次 run。stop 中止激励，但不取消已经开始的参数写入。失败原因及完整读回值可查 status 和事件。ABORTED/FINISHED 允许人工再次 run（需已确认参数且仍在 Offboard）；不存在自动下一组。

## 激励与事件

每个 repetition：`0 settle_s → +A pulse_duration_s → -A pulse_duration_s → 0 settle_s → -A pulse_duration_s → +A pulse_duration_s → 0 1.0s`。默认每轮 3.5 秒，两轮 7 秒。low/high 分别对应 `levels_deg_s` 的两项，发送时转为 rad/s。单调时钟决定脉冲时序；若调度延误跳过整段脉冲，标记 ABORTED，不补发虚构的脉冲事件。

`/pid_experiment/event` 类型为本包 `ExperimentEvent`：所需字段外增加 `trial_id`、`repetition`、`phase`（从 0 计数）、单调时钟 `steady_s` 和 `detail`。参数字段为 YAML 映射字符串，日志中为映射。事件包括全部需求事件，以及 `SESSION_START`、`PULSE_ZERO`、`PARAMETER_APPLY_FAILED`。脉冲事件时间是本机发布时刻，不宣称为飞控接收时刻；实际控制以 ULog setpoint 为证据。

退出 Offboard **不会恢复 rate PID**。操作者必须手动调用 baseline 并确认恢复成功，或降落后恢复。关闭节点也不会恢复参数。手动停止试验不会切换飞行模式。

## 记录与飞行流程

飞行数据全部由 PX4 ULog 记录。节点仅额外保存候选配置、参数读回值、试验事件和时间同步元数据，供离线分段与时钟对齐。

飞前确认 PX4 SD 卡日志启用，且 ULog 包含以下主题（可能需要调整固件 logger topic 配置）：

```
vehicle_rates_setpoint
vehicle_angular_velocity
vehicle_torque_setpoint
control_allocator_status
vehicle_attitude
vehicle_local_position
battery_status
vehicle_status
```

角速度、setpoint、torque 建议至少 200 Hz；高频分析需要高于两倍截止频率的采样率。缺失主题时分析脚本明确报错，不填造零数据。当前 ROS 1 方案无需修改 DDS topic 配置。ULog 由 PX4 自身记录；节点不自动下载日志。

1. 检查 YAML、原始 baseline 和 K0，启动 MAVROS 和试验节点，确认 TIMESYNC 数据有效。
2. 人工起飞并 Position 悬停，load 候选，等 READY。
3. 人工进入 Offboard，run low；结束后人工决定 run high。
4. 人工退出 Offboard，load 另一候选，重复同一流程。
5. 人工 baseline，确认 BASELINE_RESTORED，降落。
6. 从 SD 卡或 QGroundControl 下载本次 ULog 到会话目录，保留 ULog、YAML、事件和时间同步文件。
7. 离线分析并人工决定下一轮配置。可以依次进行 P 粗/细搜索、D 粗搜索、P/D 小网格、I、FF、pitch、yaw；程序只接受预定义列表。

## 离线分析

运行分析的 Python 环境需要 pyulog、numpy、matplotlib、PyYAML。

```bash
rosrun holydrone_autotune brct_autotune_analyze.py \
  --session /data/roll_p_coarse_01_flight01 \
  --ulog /data/roll_p_coarse_01_flight01/flight.ulg \
  --output /data/roll_p_coarse_01_flight01/report
```

`report` 应为全新目录。脚本利用节点从 MAVROS `timesync_status` 保存到 `timesync.csv` 的时钟偏移 将 ROS 事件转换到 PX4 boot 时间，逐事件插值偏移，拒绝缺失或陈旧的覆盖；还校验脉冲中央段的 ULog setpoint 方向/幅度以发现选错日志、时钟偏移或抢占发布。可用实测 `--ros-minus-px4-s` 显式覆盖偏移，不会默认用首样本对齐。此校验不是精密延迟标定；ROS 发布到 PX4 生效的传输延迟仍影响短脉冲指标。

输出每 trial 的 setpoint/actual、tracking error、torque 图及 `report.json`：

- 每个正负脉冲的 10–90% 上升时间（相对本段初始实际速率的阶跃）、目标幅度超调百分比、IAE、ITAE、tracking RMSE。
- 紧邻该脉冲的零速率窗口内回零时间（进入 ±5%A 且持续至窗口末尾至少 0.1 秒）、回零 RMS。直接反向切换的脉冲没有回零窗口，值为 null；未达到阈值也为 null。
- 实际 rate 去均值、重采样后的单边频谱高频能量，默认 `--hf-hz 30`，单位 `(rad/s)^2`；截止频率不低于 Nyquist 时为 null。短脉冲中的瞬态也贡献高频能量，不能单凭此值判断振荡。
- 所选轴 allocator `abs(unallocated_torque) > 1e-5` 的累计时间，使用零阶保持积分。
- 按候选/幅度/方向汇总所有有效重复脉冲的中位数、IQR、有效计数，另列正减负差异。中止或不完整 trial 不参与汇总，列入 excluded_trials。

不自动排名、不自动选择参数、不向飞控写回。ULog 与事件对齐检查通过不等于完成实机验收。

## 本地验证

```bash
catkin_make run_tests_holydrone_autotune
catkin_test_results build/test_results/holydrone_autotune
source devel/setup.bash
/usr/bin/python3 src/holydrone_autotune/test/autotune_mock_integration.py
```

集成测试使用独立 ROS master 和 mock 参数服务，不接触飞控。实机 200 Hz 链路吞吐、PX4 实际参数应用、遥控模式切换、ULog 主题和时钟质量仍需 SITL/台架/飞行验证。
