#!/usr/bin/env python3
import argparse
import rospy
from std_srvs.srv import Trigger
from holydrone_autotune.srv import ExperimentCommand


def main():
    parser = argparse.ArgumentParser(description='Manual rate experiment commands')
    parser.add_argument('command', choices=['load', 'run', 'stop', 'next', 'baseline', 'status'])
    parser.add_argument('value', nargs='?')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.command in ('load', 'run') and args.value is None:
        parser.error('load/run requires a candidate ID or low/high')
    rospy.init_node('brct_autotune_command', anonymous=True)
    names = dict(load='load_candidate', run='run_trial', stop='stop_trial',
                 next='next_candidate', baseline='restore_baseline', status='get_experiment_status')
    name = '/' + names[args.command]
    rospy.wait_for_service(name, timeout=5)
    typed = args.command in ('load', 'run')
    service = rospy.ServiceProxy(name, ExperimentCommand if typed else Trigger)
    result = service(args.value) if typed else service()
    print(result.message)
    if not result.success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
