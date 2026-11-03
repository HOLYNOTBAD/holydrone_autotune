#!/usr/bin/env bash
# 离线分析 Autotune 会话，默认输出到同级 <会话名>_result；结果目录必须不存在。
# 用法：./autotune_analyze.sh --session 会话名 [--ulog 日志.ulg] [--ros-minus-px4-s 偏移秒]
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${SESSION:-$ROOT/data/lite_roll_pd_02_flight09}"
CONDA_ENV="${CONDA_ENV:-holydrone_autotune}"
CONDA_EXE="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
# 留空则自动查找 <会话名>_px4 中唯一的 ULog，并使用 <会话名>_result。
ULOG="${ULOG:-}"
OUTPUT="${OUTPUT:-}"
# 留空优先使用 timesync.csv；PX4_SITL 可在全部脉冲校验后自动采用共享时钟。
# 实飞同步异常时必须显式填写实测偏移。
ROS_MINUS_PX4_S="${ROS_MINUS_PX4_S:-}"

####################### 通用函数区 #######################

####################### 启动区 #######################
if [[ ! -x "$CONDA_EXE" ]]; then
  echo "Conda executable not found: $CONDA_EXE; set CONDA_EXE to its absolute path." >&2
  exit 1
fi
ARGS=(--session "$SESSION")
[[ -z "$ULOG" ]] || ARGS+=(--ulog "$ULOG")
[[ -z "$OUTPUT" ]] || ARGS+=(--output "$OUTPUT")
[[ -z "$ROS_MINUS_PX4_S" ]] || ARGS+=(--ros-minus-px4-s "$ROS_MINUS_PX4_S")
# Conda 运行独立分析环境；Python 隔离模式避免继承 ROS 的 Python 搜索路径。
exec env -u PYTHONPATH "$CONDA_EXE" run --no-capture-output -n "$CONDA_ENV" \
  python -I "$ROOT/scripts/autotune_analyze.py" "${ARGS[@]}" "$@"
