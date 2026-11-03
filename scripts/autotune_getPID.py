#!/usr/bin/env python3
"""只读获取 PX4 当前角速度环参数，完整读取后原子保存为 baseline YAML。"""
import argparse
import math
import os
from pathlib import Path
import signal
import sys
import tempfile


TERMS = {'P': '比例项', 'I': '积分项', 'D': '微分项', 'FF': '前馈项', 'K': '总增益'}


def read_parameters(pull, get, axis):
    """刷新飞控参数表后逐项读取；任何失败均不产生部分结果。"""
    response = pull(force_pull=True)
    if not response.success or response.param_received <= 0:
        raise ValueError('飞控参数表刷新失败')
    values = {}
    for term in TERMS:
        name = 'MC_%sRATE_%s' % (axis.upper(), term)
        response = get(param_id=name)
        if not response.success:
            raise ValueError('读取参数失败：'+name)
        # MAVROS ParamValue 的 integer/real 仅其中一个承载实际值。
        value = float(response.value.integer if response.value.integer != 0 else response.value.real)
        if not math.isfinite(value) or value < 0 or (term == 'K' and value == 0):
            raise ValueError('无效参数值：%s=%s' % (name, value))
        values[name] = value
    return values


def save_baseline(path, axis, values):
    lines = ['# 从 PX4 实际读取的 %s 角速度环参数；本文件不会自动应用到飞控。' % axis,
             'baseline:  # 人工恢复使用的 P/I/D/FF/K；不会自动恢复']
    for term, description in TERMS.items():
        name = 'MC_%sRATE_%s' % (axis.upper(), term)
        # repr 保留读回数值精度，而非按候选配置或示例数值替代。
        lines.append('  %s: %r  # %s 角速度环%s' % (name, values[name], axis, description))
    content = '\n'.join(lines)+'\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.'+path.name+'.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', choices=['roll', 'pitch', 'yaw'], default='roll')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'config/brct_PID.yaml')
    parser.add_argument('--namespace', default='/mavros')
    parser.add_argument('--timeout', type=float, default=120, help='连接、刷新和读取的总超时秒数')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout 必须为有限正数')
    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import ParamGet, ParamPull

    def timed_out(_signum, _frame):
        raise TimeoutError('读取超时，请检查飞控连接、目标 system ID 和 MAVROS 日志')

    signal.signal(signal.SIGALRM, timed_out)
    signal.setitimer(signal.ITIMER_REAL, args.timeout)
    namespace = '/'+args.namespace.strip('/')
    try:
        rospy.init_node('autotune_get_pid', anonymous=True)
        print('等待飞控连接：'+namespace, flush=True)
        while not rospy.is_shutdown():
            state = rospy.wait_for_message(namespace+'/state', State, timeout=args.timeout)
            if state.connected:
                break
        if rospy.is_shutdown():
            raise RuntimeError('ROS 已关闭')
        for service in ('param/pull', 'param/get'):
            rospy.wait_for_service(namespace+'/'+service, timeout=args.timeout)
        values = read_parameters(rospy.ServiceProxy(namespace+'/param/pull', ParamPull),
                                 rospy.ServiceProxy(namespace+'/param/get', ParamGet), args.axis)
        content = save_baseline(args.output.expanduser().resolve(), args.axis, values)
        print(content, end='')
        print('已保存：'+str(args.output.expanduser().resolve()))
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print('读取 PID 失败：'+str(error), file=sys.stderr)
        raise SystemExit(1)
