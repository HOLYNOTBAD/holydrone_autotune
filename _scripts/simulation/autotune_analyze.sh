#!/usr/bin/env bash
# 离线分析调参数据；编辑参数区后运行，结果由分析程序生成。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONDA_ENV="holydrone_autotune"
SESSION="$ROOT/data/roll_pd_session" # 改为实际采集会话目录
OUTPUT="${SESSION}_result"
EXTRA_ARGS=() # 如需指定时钟偏移，可填写：(--ros-minus-px4-s 0.0)

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/autotune_analyze.py" --session "$SESSION" --output "$OUTPUT" "${EXTRA_ARGS[@]}"
