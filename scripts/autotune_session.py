#!/usr/bin/env python3
"""记录本包每个终端任务，并按 PID/启动时间停止；不依赖 ROS 或 Conda。"""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time


def start_ticks(pid):
    try:
        fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (FileNotFoundError, ProcessLookupError):
        return None


def finish(session, name):
    (session / (name+'.pending')).unlink(missing_ok=True)
    if not list(session.glob('*.pending')):
        shutil.rmtree(session, ignore_errors=True)


def run(session, name, command):
    if not command:
        raise ValueError('missing task command')
    if (session / 'stop').exists():
        finish(session, name)
        return 0
    # 每个终端任务独占进程组；关闭终端或 kill.sh 都向该组转发停止信号。
    child = subprocess.Popen(command, start_new_session=True)
    def forward(signum, _=None):
        if signum == signal.SIGHUP:
            signum = signal.SIGTERM
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, forward)
    try:
        record = session / (name+'.json')
        temporary = record.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(pid=child.pid, start_ticks=start_ticks(child.pid), command=command)))
        temporary.replace(record)
        # kill 可能发生在 Popen 与 PID 记录之间；记录后再次检查取消标志。
        if (session / 'stop').exists():
            forward(signal.SIGTERM)
        code = child.wait()
        return code if code >= 0 else 128-code
    finally:
        finish(session, name)


def stop(root, selected=None):
    targets = {}
    sessions = [selected] if selected else Path('/tmp').glob('holydrone-autotune.*')
    for session in sessions:
        try:
            if session.stat().st_uid != os.getuid() or Path((session/'package').read_text().strip()).resolve() != root:
                continue
            (session/'stop').touch()
            for path in session.glob('*.json'):
                record = json.loads(path.read_text())
                pid, ticks = record['pid'], record['start_ticks']
                if ticks and start_ticks(pid) == ticks and os.getpgid(pid) == pid:
                    targets[pid] = ticks
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, KeyError):
            continue
    if not targets:
        print('没有正在运行的本包 Autotune 任务；延迟启动的终端任务已取消。')
        return 0
    for pid, ticks in targets.items():
        if start_ticks(pid) == ticks:
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    print('已请求停止本包 Autotune 会话，等待进程清理……', flush=True)
    until = time.monotonic()+40
    while time.monotonic() < until:
        if all(start_ticks(pid) != ticks for pid, ticks in targets.items()):
            print('Autotune 会话已停止。')
            return 0
        time.sleep(.1)
    print('部分任务尚未退出，请检查该会话日志。')
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    task = commands.add_parser('run')
    task.add_argument('session', type=Path)
    task.add_argument('name')
    task.add_argument('command', nargs=argparse.REMAINDER)
    halt = commands.add_parser('stop')
    halt.add_argument('root', type=Path)
    halt.add_argument('--session', type=Path)
    args = parser.parse_args()
    if args.action == 'run':
        return run(args.session, args.name, args.command)
    return stop(args.root.resolve(), args.session)


if __name__ == '__main__':
    raise SystemExit(main())
