# holydrone_autotune SITL 验证

本包自行实现定高和回中控制，头文件位于 `include/holydrone_autotune/`。状态机负责模式交接、反馈新鲜度、激励调度与运动保护。

- `altitude_control.hpp`：高度、垂直速度反馈、有界积分、倾角补偿和推力限幅。
- `recenter_control.hpp`：水平位置/速度反馈生成倾角目标，再由姿态误差生成 FLU 角速度。

测试驱动会在隔离 ROS/Gazebo master 下执行模拟起飞、OFFBOARD 激励和降落。先构建并 source 工作空间，使用全新输出目录；本机 PX4 MAVLink 端口必须空闲。

```bash
source /opt/ros/noetic/setup.bash
source deployment/devel/setup.bash
python3 deployment/src/holydrone_autotune/test/sitl_sequence.py \
  --kind autotune --px4 /path/to/PX4-Autopilot --output /tmp/new-autotune-check
```

`--quick` 仅用于缩短排查，不代表完整默认配置通过。迁移前的历史结果保存在 `test/results/`；这些数据来自单次无风仿真，不代表本次修改重新执行了 SITL。
