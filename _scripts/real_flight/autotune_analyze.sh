#!/usr/bin/env bash
# 离线分析调参数据；编辑参数区后运行，结果由分析程序生成。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONDA_ENV="holydrone_autotune"
# 默认分析最新会话，可传入实验目录或会话名。
unset PYTHONPATH
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/autotune_analyze.py" "$@"
