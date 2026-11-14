#!/usr/bin/env bash
# 启动 roll 轴调参节点并记录 PID；通过同目录 kill.sh 停止。采集结果由节点保存。
# 同一窗口分 Tab 显示任务输出；主入口直接运行，无需传参。
set -euo pipefail

####################### 参数区 #######################
SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="$(cd -- "$ROOT/../.." && pwd)"
PID_FILE="/tmp/holydrone-autotune-real_flight-$(printf '%s' "$ROOT" | sha256sum | cut -c1-16).pids"
CONFIG="$ROOT/config/brct_autotune_roll_trial2.yaml"
OUTPUT="$ROOT/data/experiment/roll_pd_trial2_$(date +%Y%m%d_%H%M%S)_$$"
ROS_PORT=11311
FCU_URL="/dev/ttyS0:2000000"
TARGET_SYSTEM=2
DOWNLOAD_ULOG=true

####################### 环境变量区 #######################
unset PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CONDA_PREFIX ROS_HOSTNAME ROS_NAMESPACE
set +u
source "$WORKSPACE/devel/setup.bash" --
set -u
export PATH="/opt/ros/noetic/bin:/usr/local/bin:/usr/bin:/bin" # 使用系统 ROS 命令与解释器
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT" # 本次任务使用的 ROS Master 地址
export ROS_IP=127.0.0.1 # 节点通告本机回环地址
export MAVROS_FCU_URL="$FCU_URL" # mavros_real.launch 使用的飞控连接地址
export MAVROS_TGT_SYSTEM="$TARGET_SYSTEM" # MAVROS 的目标 system ID

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
  --command="$(terminal_command roslaunch holydrone mavros_real.launch)" \
  --tab --title="Autotune" \
  --command="$(terminal_command roslaunch --wait holydrone_autotune brct_autotune.launch \
    "config:=$CONFIG" "output:=$OUTPUT" "download_ulog:=$DOWNLOAD_ULOG")"
