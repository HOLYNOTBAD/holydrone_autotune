#!/usr/bin/env bash
# 启动 pitch 轴调参节点并记录 PID；通过同目录 kill.sh 停止。采集结果由节点保存。
# 同一窗口分 Tab 显示任务输出；主入口直接运行，无需传参。
set -euo pipefail

####################### 参数区 #######################
SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="$(cd -- "$ROOT/../.." && pwd)"
PID_FILE="/tmp/holydrone-autotune-simulation-$(printf '%s' "$ROOT" | sha256sum | cut -c1-16).pids"
CONFIG="$ROOT/config/brct_autotune_pitch.yaml"
OUTPUT="$ROOT/data/experiment/pitch_pd_$(date +%Y%m%d_%H%M%S)_$$"
ROS_PORT=11311
GAZEBO_SETUP="$HOME/.local/share/gazebo_ros_ws/devel/setup.bash"
PX4_DIR="$HOME/PX4-Autopilot-holydrone-lite"
WORLD="$WORKSPACE/src/holydrone_ros/gazebo/worlds/empty.world"
AIRCRAFT="holydrone_lite"
SPAWN_Z=0.055
DOWNLOAD_ULOG=false

####################### 环境变量区 #######################
unset PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CONDA_PREFIX ROS_HOSTNAME ROS_NAMESPACE
set +u
source "$WORKSPACE/devel/setup.bash" --
source "$GAZEBO_SETUP" --extend
set -u
export PATH="/opt/ros/noetic/bin:/usr/local/bin:/usr/bin:/bin" # 使用系统 ROS 命令与解释器
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT" # 本次任务使用的 ROS Master 地址
export ROS_IP=127.0.0.1 # 节点通告本机回环地址
export GAZEBO_MODEL_DATABASE_URI="" # Gazebo 使用本地模型库

####################### 通用函数区 #######################
terminal_command() {
  printf '%q ' bash "$SCRIPT" --run-tab "$@"
}

####################### 启动区 #######################
# --run-tab 仅供本脚本创建的 Tab 内部调用；每页加载环境后执行一个任务。
if [[ "${1-}" == --run-tab ]]; then
  shift
  set +m
  setsid "$@" < /dev/null &
  pid=$!
  printf '%s\n' "$pid" >> "$PID_FILE"
  wait "$pid"
  exit
fi

gnome-terminal \
  --window --title="PX4 + Gazebo + MAVROS" \
  --command="$(terminal_command roslaunch holydrone sitl.launch \
    "aircraft:=$AIRCRAFT" "px4_dir:=$PX4_DIR" start_mavros:=true \
    "px4_runtime_dir:=${OUTPUT}_px4" \
    "world:=$WORLD" "spawn_z:=$SPAWN_Z")" \
  --tab --title="Autotune" \
  --command="$(terminal_command roslaunch --wait holydrone_autotune brct_autotune.launch \
    "config:=$CONFIG" "output:=$OUTPUT" "download_ulog:=$DOWNLOAD_ULOG")"
