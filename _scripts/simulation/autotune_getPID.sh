#!/usr/bin/env bash
# 启动 MAVROS 并读取飞控 PID 参数；输出由读取程序保存，用 kill.sh 停止任务。
# 同一窗口分 Tab 显示任务输出；主入口直接运行，无需传参。
set -euo pipefail

####################### 参数区 #######################
SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="$(cd -- "$ROOT/../.." && pwd)"
PID_FILE="/tmp/holydrone-autotune-simulation-$(printf '%s' "$ROOT" | sha256sum | cut -c1-16).pids"
AXIS="roll"
OUTPUT="$ROOT/config/brct_PID.yaml"
TIMEOUT=120
ROS_PORT=11311
FCU_URL="udp://:14540@127.0.0.1:14557"
TARGET_SYSTEM=1

####################### 环境变量区 #######################
unset PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CONDA_PREFIX ROS_HOSTNAME ROS_NAMESPACE
set +u
source "$WORKSPACE/devel/setup.bash" --
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
  --window --title="MAVROS" \
  --command="$(terminal_command roslaunch mavros px4.launch "fcu_url:=$FCU_URL" gcs_url:= "tgt_system:=$TARGET_SYSTEM")" \
  --tab --title="Read PID" \
  --command="$(terminal_command /usr/bin/python3 "$ROOT/scripts/autotune_getPID.py" \
    --axis "$AXIS" --output "$OUTPUT" --timeout "$TIMEOUT")"
