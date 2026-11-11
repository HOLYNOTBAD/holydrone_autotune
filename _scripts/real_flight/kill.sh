#!/usr/bin/env bash
# 读取本目录入口记录的 PID，停止对应进程组并删除 PID 文件。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PID_FILE="/tmp/holydrone-autotune-real_flight-$(printf '%s' "$ROOT" | sha256sum | cut -c1-16).pids"

####################### 环境变量区 #######################

####################### 通用函数区 #######################

####################### 启动区 #######################
mapfile -t pids < "$PID_FILE"
for ((i=${#pids[@]}-1; i>=0; i--)); do
  kill -TERM -- "-${pids[i]}" || true
done
rm -- "$PID_FILE"
