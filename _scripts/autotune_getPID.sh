#!/usr/bin/env bash
# 启动 MAVROS、读取当前 PID 并保存；成功/失败/Ctrl+C 均清理本次启动的 MAVROS。
# 用法：MODE=simulation 或 real_flight ./autotune_getPID.sh [--axis roll]
# simulation 连接已运行的 PX4 SITL；本脚本仅启动 MAVROS，不启动仿真或试验。
set -e

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${MODE:-real_flight}"  # simulation | real_flight
SIM_FCU_URL="udp://:14540@127.0.0.1:14557"
REAL_FCU_URL="/dev/ttyS0:2000000"
OUTPUT="${OUTPUT:-$ROOT/config/brct_PID.yaml}"
TIMEOUT="${TIMEOUT:-120}"

####################### 通用函数区 #######################
cleanup() {
  trap '' INT TERM
  if [[ -n "${READER_PID:-}" ]]; then
    kill -TERM "$READER_PID" 2>/dev/null || true
    wait "$READER_PID" 2>/dev/null || true
  fi
  if [[ -n "${MAVROS_PID:-}" ]]; then
    kill -INT "$MAVROS_PID" 2>/dev/null || true
    wait "$MAVROS_PID" 2>/dev/null || true
  fi
}

####################### 启动区 #######################
case "$MODE" in
  simulation)
    FCU_URL="${MAVROS_FCU_URL:-$SIM_FCU_URL}"
    TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-1}"
    ;;
  real_flight)
    FCU_URL="${MAVROS_FCU_URL:-$REAL_FCU_URL}"
    TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-2}"
    ;;
  *) echo "Unsupported MODE: $MODE; use simulation or real_flight." >&2; exit 1 ;;
esac
source /opt/ros/noetic/setup.bash
# 不替换已运行的同名节点，避免关闭用户原有的 MAVROS。
if rosnode list 2>/dev/null | grep -qx '/mavros'; then
  echo 'MAVROS 已运行；可直接运行 scripts/autotune_getPID.py 读取。' >&2
  exit 1
fi
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "MODE=$MODE FCU_URL=$FCU_URL TGT_SYSTEM=$TGT_SYSTEM"
roslaunch mavros px4.launch "fcu_url:=$FCU_URL" gcs_url:= "tgt_system:=$TGT_SYSTEM" &
MAVROS_PID=$!
/usr/bin/python3 "$ROOT/scripts/autotune_getPID.py" --output "$OUTPUT" --timeout "$TIMEOUT" "$@" &
READER_PID=$!
wait "$READER_PID"
