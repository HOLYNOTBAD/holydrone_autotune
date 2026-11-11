# holydrone_autotune SITL 验证

回中由 PX4 OFFBOARD 位置控制完成；只有角速度脉冲期间使用本包的 `altitude_control.hpp`。状态机通过零角速度交接在位置与角速度指令之间互斥切换，每个四脉冲组前重新采集稳定位置控制推力。

正常结束、回中超时及人工停止都保持 OFFBOARD 位置控制；反馈失效时停止发送，PX4 自身失联保护仍有效。

测试驱动会在隔离 ROS/Gazebo master 下执行模拟起飞、OFFBOARD 激励和降落。先构建并 source 工作空间，使用全新输出目录；本机 PX4 MAVLink 端口必须空闲。

```bash
source /opt/ros/noetic/setup.bash
source deployment/devel/setup.bash
python3 deployment/src/holydrone_autotune/test/sitl_sequence.py \
  --kind autotune --px4 /path/to/PX4-Autopilot \
  --config deployment/src/holydrone_autotune/config/brct_autotune_sim.yaml \
  --output /tmp/new-autotune-check
```

`--quick` 仅用于缩短排查，不代表完整默认配置通过。迁移前的历史结果保存在 `test/results/`；这些数据来自单次无风仿真，不代表本次修改重新执行了 SITL。

测试驱动验证试验及完成后的 3 秒保持期间模式始终为 OFFBOARD，然后由测试驱动请求 AUTO.LAND 清理。`--expect-abort` 配合严格的回中门槛配置，验证超时保护后保持 OFFBOARD。

## 逐脉冲回中版本验证（历史）

本次运行使用 holydrone_lite 和仿真配置的前两组候选，保留 60/120°/s、0.25 s 脉冲与原回中门槛；不是全部 25 组候选的验证。

- 完成 4 档、16 次脉冲；ULog 确认脉冲期间为角速度控制，回中使用 PX4 位置控制，试验及完成后保持阶段均为 OFFBOARD。
- 最大水平偏移 2.34 m；所有回中判定均满足 0.75 m 水平、0.15 m 高度、0.3 m/s 速度门槛。最后一次回中水平误差 0.312 m、高度误差约 0.001 m。
- 最大掉高 0.043 m，最大倾角 32.13°。
- 故意把水平门槛设为 1 µm、超时设为 2 s：触发回中超时且不发脉冲，随后仍保持 OFFBOARD 位置控制至少 3 s。
- 7 项 C++、21 项 Python 单元测试和模拟 FCU 接口集成测试通过。Python 分析测试使用包的 Conda 环境；系统 Python 缺少 pyulog。
- 原测试驱动把回中成功事件数写成固定值，但零角速度交接重检失败会再次回中，造成额外成功事件；已改为检查全部脉冲数和每次回中门槛，并对本次 ULog 完成回放校验。

汇总数据见 [px4_position_recovery.json](results/px4_position_recovery.json)。仿真使用独立 master 和 runtime，完成后由测试驱动降落并上锁；未部署到实机。

## 连续四脉冲组版本验证

组内连续发送 `+−−+`，每个脉冲 0.25 s；四脉冲之间不切位置控制、不插零角速度、不重新采集推力。组末才执行零角速度交接与 PX4 位置回中，下一组需重新满足稳定条件。

Gazebo 使用仿真配置前两组候选，每组候选 low/high 两档，每档 repetitions=2：共 8 组、32 次脉冲，全部完成。ULog 验证所有组内保持角速度控制、指令连续，整段试验及完成保持阶段始终 OFFBOARD；12 次回中判定（4 次档位初始稳定 + 8 次组末回中）均通过。

最大水平偏移 0.748 m、最大掉高 0.005 m、最大倾角 29.17°。22 项 Python 单元测试（包括连续四脉冲离线解析）和模拟接口集成测试通过。第一次仿真在激励前因 D 参数读回不一致被正确阻止；全新 runtime 重测完成上述序列，没有放宽参数核验。

结果见 [group_position_recovery.json](results/group_position_recovery.json)。仅修改本机代码，未将四脉冲组版本部署到飞机；仿真结果不代表实机所有候选都会通过回中门槛。

## 参数链路与控制回调隔离回归

以下测试使用隔离 ROS master 和模拟飞控，不连接串口。先构建节点及专用只读探针，再运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C deployment --pkg holydrone_autotune
cmake --build deployment/build --target parameter_link_probe -j2
source deployment/devel/setup.bash
/usr/bin/python3 deployment/src/holydrone_autotune/test/autotune_mock_integration.py
```

探针故意不处理默认 ROS 回调队列，验证参数回复线程能独立运行。回归还覆盖错误来源、旧时间戳、丢包重试、不重复写未变化项，以及完整候选激励序列。


### 脉冲间 PX4 水平姿态恢复（2026-11-05）

每个脉冲后先发 0.1 s 零角速度，再由 PX4 姿态环恢复水平并保持航向；倾角 ≤3°、三轴角速度合量 ≤5°/s 连续 0.3 s 后放行，等待上限 8 s。组内下一脉冲前保留至少 0.65 s 零角速度交接。四脉冲后统一位置回中。

Gazebo 单候选、60/120 deg/s、0.25 s 共 8 脉冲完成，始终 OFFBOARD，结束保持 3 s。最大倾角 34.40°、水平偏移 5.55 m、高度下降 0.0474 m；结果见 `test/results/level_attitude_recovery.json`。原始记录 `/tmp/level-pause-sitl2`。脚本中已移除与当前控制器不一致的旧回中速度断言，新增每次脉冲等待结束的水平/角速度检查，并对原始记录重新执行全部断言通过。首次无刹车缓冲的试验超出 45° 限值，不能视作通过。

模拟 ROS 通信回归通过，覆盖角速度归零但倾斜仍阻止后续激励、超时保护和整组位置回中。此验证不是实飞验证，也未遍历全部候选。


### 水平恢复交接重试

实飞事件暴露：姿态恢复成功后切入零角速度，若倾角再次超过门限，旧逻辑只等待直至超时。修复为交接至少 0.65 s 后若新鲜反馈/稳定窗口不满足，重启 PX4 水平姿态控制并重新稳定；整个等待仍受 8 s 总时限约束。

模拟通信回归通过，新增交接后人为倾斜的拒绝、姿态控制恢复和后续脉冲检查。Gazebo `/tmp/handover-retry-sitl` 完成单候选两档 8 脉冲，实际触发交接拒绝后恢复并继续完成，原始结果见 `test/results/handover_retry_recovery.json`。高度回中 0.15 m 门限未改变，独立的高度回中超时不由此修复消除。


### 当前策略：角速度稳定与 0.5 m 高度容差

按用户要求取消绝对倾角门限，仍发 PX4 水平姿态目标；放行只需三轴角速度合量 ≤5 deg/s 连续新鲜反馈 0.3 s，8 s 总时限及交接失败重试保持。回中高度容差 0.5 m。模拟通信测试验证固定 8° 倾角可放行，角速度扰动仍拒绝交接。Gazebo `/tmp/stable-rate-sitl` 单候选两档共 8 脉冲完成，全程 OFFBOARD，结束保持 3 s，结果见 `test/results/rate_stability_recovery.json`；不是全候选或实飞验证。


### 分离切换缓冲与稳定观察窗口

实飞第二候选出现交接在 0.65 s 时立即拒绝、反复重试耗尽 8 s。修复：0.65 s 内不累计放行稳定窗口；随后留至少 1 s 观察角速度连续稳定（0.3 s），再判交接失败，8 s 总时限不变。新增模拟通信测试以 0.8 s 切换扰动验证不会提前拒绝，稳定后可以继续；完整回归通过。Gazebo `/tmp/handover-window-sitl` 两个候选、两档共 16 个脉冲完成，结果见 `test/results/handover_window_recovery.json`。这不保证所有实飞候选都满足稳定条件。


### 当前序列：+- 双脉冲组与分轴启动

每档每 repetition 改为正、负各一次；稳定等待、控制交接、整组位置回中保持原逻辑。Gazebo `/tmp/pair-sitl` 两个 roll 候选、两档共 8 个脉冲完成，全程 OFFBOARD，完成后保持 3 s；结果见 `test/results/two_pulse_recovery.json`。16 个离线测试与完整模拟通信回归通过，后者检查双脉冲顺序和回中。三个启动脚本通过 bash 语法和占位终端参数验证（含空格路径、轴匹配拒绝），没有实际启动 pitch/yaw 飞行。pitch/yaw 配置来自本次 FCU 只读快照；不是 pitch/yaw 实飞或全网格仿真验证。
