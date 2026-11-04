#!/usr/bin/env bash
# 在同一窗口的不同标签页启动 MAVROS/SITL 与自动候选试验节点。
# MODE=simulation 或 real_flight；需要 GNOME Terminal 和 Catkin。
# OUTPUT 为全新会话目录；不自动起飞，不自动恢复 baseline；停止任务使用同目录 kill.sh。
set -e

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${WORKSPACE:-$(cd -- "$ROOT/../.." && pwd)}"
SETUP="$WORKSPACE/devel/setup.bash"
[[ -f "$SETUP" ]] || SETUP="$WORKSPACE/setup.bash"
WORLD="$WORKSPACE/src/holydrone_ros/gazebo/worlds/empty.world"
[[ -f "$WORLD" ]] || WORLD="$WORKSPACE/share/holydrone/gazebo/worlds/empty.world"
MODE="${MODE:-real_flight}"  # simulation | real_flight
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-holydrone-lite}"
DOWNLOAD_ULOG=false
[[ "$MODE" != "real_flight" ]] || DOWNLOAD_ULOG=true
# 默认创建独立的 lite 仿真会话；也可用 OUTPUT 指定尚不存在的目录。
OUTPUT="${OUTPUT:-$ROOT/data/lite_roll_pd_01_$(date +%Y%m%d_%H%M%S)}"

####################### 通用函数区 #######################
launch_tab() {
  local title="$1" command
  shift
  local args=(env "PX4_DIR=$PX4_DIR" "MAVROS_FCU_URL=${MAVROS_FCU_URL:-/dev/ttyS0:2000000}"
    "MAVROS_TGT_SYSTEM=${MAVROS_TGT_SYSTEM:-2}"
    "ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" /usr/bin/python3 "$ROOT/scripts/autotune_session.py" run "$SESSION_DIR" "$title"
    bash -c 'set -e; source /opt/ros/noetic/setup.bash; source "$1"; shift; exec roslaunch "$@"' bash "$SETUP" "$@")
  touch "$SESSION_DIR/$title.pending"
  # 按终端的命令解析规则引用各参数，保留空格与单引号。
  printf -v command "'%s' " "${args[@]//\'/\'\\\'\'}"
  TERMINAL+=("$TAB" --title="$title" --working-directory="$WORKSPACE" --command="$command")
  TAB=--tab
}

####################### 启动区 #######################
[[ "$MODE" == simulation || "$MODE" == real_flight ]] || { echo "Invalid MODE: $MODE" >&2; exit 2; }
if [[ "$MODE" == simulation ]]; then
  # 兼容迁移前已打开的终端/IDE；仅替换已失效的旧默认路径。
  if [[ "$PX4_DIR" == "$HOME/PX4-Autopilot" && ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" &&
        -x "$HOME/PX4-Autopilot-holydrone-lite/build/px4_sitl_default/bin/px4" ]]; then
    echo "旧 PX4_DIR 已失效，使用 $HOME/PX4-Autopilot-holydrone-lite" >&2
    PX4_DIR="$HOME/PX4-Autopilot-holydrone-lite"
  fi
  [[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" && -d "$PX4_DIR/build/px4_sitl_default/etc" ]] || {
    echo "PX4 SITL build not found in $PX4_DIR; set PX4_DIR to the built lite checkout." >&2
    exit 1
  }
  echo "PX4_DIR: $PX4_DIR"
fi
if [[ -e "$OUTPUT" ]]; then
  echo "Output directory already exists: $OUTPUT; set OUTPUT to a new session directory." >&2
  exit 1
fi
[[ -f "$SETUP" ]] || { echo "Workspace setup not found: $SETUP" >&2; exit 1; }
command -v gnome-terminal >/dev/null
SESSION_DIR="$(mktemp -d /tmp/holydrone-autotune.XXXXXX)"
printf '%s\n' "$ROOT" > "$SESSION_DIR/package"
TERMINAL=(gnome-terminal)
TAB=--window
if [[ "$MODE" == "simulation" ]]; then
  launch_tab "PX4-Gazebo" holydrone sitl.launch \
    aircraft:=holydrone_lite \
    start_mavros:=true \
    "px4_runtime_dir:=${OUTPUT}_px4" \
    "world:=$WORLD" \
    spawn_z:=0.055
else
  launch_tab "MAVROS" holydrone mavros_real.launch
fi
# --wait 等待首个 launch 创建 ROS Master 和 /run_id。
launch_tab "Autotune" --wait holydrone_autotune brct_autotune.launch \
  "output:=$OUTPUT" "download_ulog:=$DOWNLOAD_ULOG"
if ! "${TERMINAL[@]}"; then
  /usr/bin/python3 "$ROOT/scripts/autotune_session.py" stop "$ROOT" --session "$SESSION_DIR"
  exit 1
fi
