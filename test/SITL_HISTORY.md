# 控制拆分前的 SITL 验证记录

以下为控制拆分前的历史验证。当前两个包各自包含独立控制器和测试入口；本文数字不表示本次拆分重新运行 SITL。

- `altitude_control.hpp`：进入 Offboard 前捕获的悬停推力，叠加高度/垂直速度反馈、有界积分和倾角补偿；输出限幅并抑制积分饱和。
- `recenter_control.hpp`：ENU 水平位置/速度反馈生成有限倾角目标，再通过四元数姿态误差生成 FLU 角速度。全程保持 body-rate 控制类型。

两包各自的 FSM 负责新鲜度、模式交接、脉冲调度与越界退出。autotune 另负责候选参数事务；identification 的两个 `*_excite.cpp` 仅生成波形，由 `identification_fsm.cpp` 统一发布和回中。控制器由各包状态机调用。

默认回中条件：水平偏移 ≤0.75 m、高度误差 ≤0.2 m、三维速度 ≤0.5 m/s、倾角 ≤8°、航向误差 ≤5°，连续稳定 0.2 s。不是无条件等待 0.2 s；未稳定就继续闭环回中，8 s 超时则中止。激励窗口不叠加姿态回中角速度，以保留清晰的辨识输入。

物理辨识的四个耦合方向各配一段反向输入。实测原四个连续符号组合会在回中前积累约 3.9 m 水平偏移，改成双脉冲组后本次测试最大偏移约 0.88 m。推力辨识的短双脉冲冻结高度控制基准及积分，组后恢复高度闭环，避免抵消辨识输入。

## 本次闭环验证

使用本机 PX4 SITL、Holydrone 模型、empty world、默认配置完整序列；均从约 3.5 m Position 悬停手动模式交接。测试驱动自动执行模拟起飞/Offboard/降落，不接触真实飞控。

| 流程 | 脉冲数 | 整套耗时 | 最大掉高 | 最大水平偏移 | 最大倾角 |
| --- | ---: | ---: | ---: | ---: | ---: |
| autotune：3 候选 × low/high × 2 轮 | 48 | 53.96 s | 0.020 m | 1.453 m | 40.41° |
| 一阶辨识：4 通道 × 3 轮 | 24 | 13.00 s | 0.035 m | 0.358 m | 4.81° |
| 物理辨识：含反向耦合组 × 2 轮 | 48 | 40.03 s | 0.071 m | 0.883 m | 16.02° |

全部完成后由飞控状态确认退出到 POSCTL。数字是单次无风仿真结果，尚未验证真实机型、风扰、不同载荷或不同候选增益。高幅值角速度激励仍会产生明显倾斜，运动保护默认上限为 45°。

autotune 结果摘要保存在本目录 `results/autotune.json`；辨识摘要保存在 `holydrone_identification/test/results/`。测试原始输出分别在 `/tmp/autotune-paired-final`、`/tmp/brct1order-paired-final`、`/tmp/physics-paired-sitl-03`，包含配置、逐阶段记录、位姿采样及 PX4 ULog。

## 复现

先构建并 source 工作空间。下面会启动隔离的 ROS/Gazebo master，但 PX4 使用固定的本机 MAVLink 端口；端口占用时测试拒绝启动，不要与另一套 SITL 同时运行。每次使用全新输出目录：

```bash
source /opt/ros/noetic/setup.bash
source deployment/devel/setup.bash
python3 deployment/src/holydrone_autotune/test/sitl_sequence.py \
  --kind autotune --px4 /path/to/PX4-Autopilot --output /tmp/new-autotune-check
# 一阶/物理辨识请使用 holydrone_identification/test/sitl_sequence.py。
# --quick 仅用于缩短排查，不代表默认完整序列通过。
```

还验证了：

- Catkin 编译两个 FSM 及公共头文件依赖。
- 7 项 C++ 测试：高度控制、限幅/抗饱和、反馈坐标方向、四元数符号不变性、波形配平。
- autotune 模拟 FCU 集成：参数失败、模式交接、完整候选、组间回中、水平门控、旧反馈和越界退出。
- identification 模拟 FCU 集成：两套完整波形、基准推力锁定、恢复门控、旧时间戳、倾角/高度/回中超时和人工停止；慢模式服务不阻塞指令流。
- 10 项 Python 离线测试；一阶辨识实际 SITL ULog 可提取 roll/pitch/yaw/thrust 各 3 组双脉冲。

未运行依赖已移走 autotune 脚本的端到端离线分析/ULog 下载测试；脚本目录迁移保持使用者当前状态。
