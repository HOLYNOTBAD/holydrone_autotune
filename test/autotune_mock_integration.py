#!/usr/bin/env python3
"""Run explicitly after sourcing devel/setup.bash. Uses its own ROS master, no FCU."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import yaml


class MockFlight(unittest.TestCase):
    def test_manual_flight(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
        processes = []
        with tempfile.TemporaryDirectory() as tmp:
            log = open(Path(tmp)/'ros.log', 'w')
            try:
                processes.append(subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log, start_new_session=True))
                import rospy
                import rosgraph
                deadline = time.monotonic()+15
                while not rosgraph.is_master_online():
                    if time.monotonic() > deadline:
                        self.fail('master startup timed out')
                    time.sleep(.1)
                from mavros_msgs.msg import State, AttitudeTarget, ParamValue, TimesyncStatus
                from mavros_msgs.srv import ParamSet, ParamSetResponse, ParamGet, ParamGetResponse, ParamPull, ParamPullResponse
                from std_srvs.srv import Trigger
                from holydrone_autotune.srv import ExperimentCommand
                from holydrone_autotune.msg import ExperimentEvent
                rospy.init_node('autotune_mock_test', disable_signals=True)
                values, commands, events = {}, [], []
                faults = {'write': False, 'mismatch': False, 'pulls': 0}
                def set_param(req):
                    if faults['write']:
                        return ParamSetResponse(False, ParamValue())
                    values[req.param_id] = req.value.real
                    return ParamSetResponse(True, req.value)
                def get_param(req):
                    return ParamGetResponse(True, ParamValue(real=values[req.param_id]+(.1 if faults['mismatch'] else 0)))
                def pull(req):
                    self.assertTrue(req.force_pull); faults['pulls'] += 1
                    return ParamPullResponse(True, len(values))
                servers = [rospy.Service('/mavros/param/set', ParamSet, set_param),
                           rospy.Service('/mavros/param/get', ParamGet, get_param),
                           rospy.Service('/mavros/param/pull', ParamPull, pull)]
                subscribers = [rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget,
                                               lambda m: commands.append((time.monotonic(), m))),
                               rospy.Subscriber('/pid_experiment/event', ExperimentEvent, events.append)]
                state = State(connected=True, mode='POSCTL')
                pub = rospy.Publisher('/mavros/state', State, queue_size=1)
                sync_pub = rospy.Publisher('/mavros/timesync_status', TimesyncStatus, queue_size=1)
                def feedback(_):
                    pub.publish(state)
                    sync = TimesyncStatus()
                    sync.header.stamp = rospy.Time.now()
                    sync.estimated_offset_ns = 1700000000000000000
                    sync_pub.publish(sync)
                timer = rospy.Timer(rospy.Duration(.05), feedback)
                root = Path(__file__).resolve().parents[1]
                config = yaml.safe_load((root/'config/brct_autotune.yaml').read_text())
                # Exercise pitch sign conversion as well as the full FSM.
                config['axis'] = 'pitch'
                config['baseline'] = {k.replace('ROLL', 'PITCH'): v for k, v in config['baseline'].items()}
                config['candidates'] = [{k.replace('ROLL', 'PITCH'): v for k, v in c.items()} for c in config['candidates']]
                config['excitation'].update(settle_s=.1, pulse_duration_s=.08, repetitions=1)
                path = Path(tmp)/'config.yaml'; path.write_text(yaml.safe_dump(config))
                processes.append(subprocess.Popen(['roslaunch', 'holydrone_autotune', 'brct_autotune.launch',
                    'config:='+str(path), 'output:='+str(Path(tmp)/'session')], stdout=log, stderr=log, start_new_session=True))
                rospy.wait_for_service('/load_candidate', timeout=15)
                load = rospy.ServiceProxy('/load_candidate', ExperimentCommand)
                run = rospy.ServiceProxy('/run_trial', ExperimentCommand)
                status = rospy.ServiceProxy('/get_experiment_status', Trigger)
                stop = rospy.ServiceProxy('/stop_trial', Trigger)
                baseline = rospy.ServiceProxy('/restore_baseline', Trigger)
                next_candidate = rospy.ServiceProxy('/next_candidate', Trigger)
                def wait_for(fn, timeout=8):
                    end = time.monotonic()+timeout
                    while not fn():
                        if time.monotonic() > end:
                            self.fail('condition timeout; '+status().message)
                        time.sleep(.01)
                self.assertFalse(run('low').success)
                self.assertFalse(load('unknown').success)
                self.assertTrue(load('p_1p00').success)
                self.assertFalse(load('p_0p90').success)
                wait_for(lambda: 'state=READY' in status().message)
                self.assertGreater(faults['pulls'], 0)
                self.assertFalse(run('low').success)
                self.assertTrue(all(m.body_rate.y == 0 for _, m in commands))
                state.mode = 'OFFBOARD'; wait_for(lambda: 'offboard=true' in status().message)
                self.assertTrue(run('low').success)
                self.assertFalse(run('high').success)
                self.assertFalse(baseline().success)
                wait_for(lambda: any(m.body_rate.y < -.5 for _, m in commands))
                state.mode = 'POSCTL'
                wait_for(lambda: 'state=ABORTED' in status().message)
                time.sleep(.08)
                self.assertEqual(commands[-1][1].body_rate.y, 0)
                self.assertTrue(any(e.event_type == 'TRIAL_ABORTED' for e in events))
                state.mode = 'OFFBOARD'; wait_for(lambda: 'offboard=true' in status().message)
                self.assertTrue(run('high').success)
                wait_for(lambda: 'state=FINISHED' in status().message)
                pulse_types = [e.event_type for e in events if e.trial_id == 2 and e.event_type.startswith('PULSE_')]
                self.assertEqual(pulse_types, ['PULSE_ZERO', 'PULSE_POSITIVE', 'PULSE_NEGATIVE', 'PULSE_ZERO',
                                              'PULSE_NEGATIVE', 'PULSE_POSITIVE', 'PULSE_ZERO'])
                self.assertTrue(run('low').success); self.assertTrue(stop().success)
                self.assertIn('state=ABORTED', status().message)
                self.assertTrue(next_candidate().success); wait_for(lambda: 'state=READY' in status().message)
                self.assertIn('candidate=p_1p10', status().message)
                faults['mismatch'] = True
                self.assertTrue(load('p_0p90').success)
                wait_for(lambda: 'state=IDLE' in status().message)
                self.assertFalse(run('low').success)
                faults['mismatch'] = False; faults['write'] = True
                self.assertTrue(load('p_1p00').success); wait_for(lambda: 'state=IDLE' in status().message)
                self.assertFalse(run('low').success)
                faults['write'] = False
                self.assertTrue(baseline().success); wait_for(lambda: 'state=READY' in status().message)
                self.assertTrue(any(e.event_type == 'BASELINE_RESTORED' for e in events))
                self.assertEqual(values, config['baseline'])
                # Parameter transfer/wait must not interrupt streaming.
                gaps = [b[0]-a[0] for a, b in zip(commands, commands[1:])]
                self.assertLess(max(gaps), .2)
                self.assertGreater(len(commands), 1000)
                timer.shutdown()
                launch = processes.pop()
                os.killpg(launch.pid, signal.SIGINT); launch.wait(timeout=15)
                session = Path(tmp)/'session'
                self.assertFalse(list(session.glob('*.bag*')))
                entries = list(yaml.safe_load_all((session/'events.yaml').read_text()))
                self.assertGreater(len(entries), 15)
                import csv
                with (session/'timesync.csv').open() as stream:
                    samples = list(csv.DictReader(stream))
                self.assertGreater(len(samples), 10)
                self.assertEqual(float(samples[-1]['ros_minus_px4_s']), 1700000000.)
            finally:
                for proc in reversed(processes):
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL); proc.wait()
                log.close()


if __name__ == '__main__':
    unittest.main()
