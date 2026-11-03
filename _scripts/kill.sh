#!/usr/bin/env bash
# 停止本包启动的 Autotune 会话；按记录的 PID 和启动时间识别本次任务。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

####################### 通用函数区 #######################

####################### 启动区 #######################
exec /usr/bin/python3 "$ROOT/scripts/autotune_session.py" stop "$ROOT"
