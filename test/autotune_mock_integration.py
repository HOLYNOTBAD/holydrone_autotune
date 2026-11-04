#!/usr/bin/env python3
"""使用独立 ROS Master 和模拟 FCU 参数服务验证服务调试与自动候选序列。
显式运行前需 source devel/setup.bash；不连接真实飞控。"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import yaml
import math


class MockFlight(unittest.TestCase):
    """提供独立 ROS Master、模拟参数服务和状态反馈，收集节点指令与事件。"""
    def test_automatic_flight(self):
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
                from mavros_msgs.msg import State, AttitudeTarget, PositionTarget, ParamValue, TimesyncStatus
                from geometry_msgs.msg import PoseStamped, TwistStamped
                from sensor_msgs.msg import Imu
                from mavros_msgs.srv import SetMode, SetModeResponse, ParamSet, ParamSetResponse, ParamGet, ParamGetResponse, ParamPull, ParamPullResponse
                from std_srvs.srv import Trigger
                from holydrone_autotune.srv import ExperimentCommand
                from holydrone_autotune.msg import ExperimentEvent
                rospy.init_node('autotune_mock_test', disable_signals=True)
                values, commands, events = {}, [], []
                position_commands, stream_times = [], []
                def attitude_command(m):
                    commands.append((time.monotonic(), m)); stream_times.append(time.monotonic())
                def position_command(m):
                    position_commands.append((time.monotonic(), m)); stream_times.append(time.monotonic())
                faults = {'write': False, 'mismatch': False, 'pulls': 0, 'cache_ready': False}
                def set_param(req):
                    if faults['write'] or not faults['cache_ready']:
                        return ParamSetResponse(False, ParamValue())
                    values[req.param_id] = req.value.real
                    return ParamSetResponse(True, req.value)
                def get_param(req):
                    if not faults['cache_ready'] or req.param_id not in values:
                        return ParamGetResponse(False, ParamValue())
                    return ParamGetResponse(True, ParamValue(real=values[req.param_id]+(.1 if faults['mismatch'] else 0)))
                def pull(req):
                    self.assertTrue(req.force_pull); faults['pulls'] += 1
                    # Match MAVROS startup: heartbeat can precede the parameter cache.
                    time.sleep(.1)
                    faults['cache_ready'] = True
                    return ParamPullResponse(True, len(values))
                mode_requests = []
                def set_mode(req):
                    mode_requests.append(req.custom_mode)
                    # Slow service must not stall setpoint streaming. Accept the second
                    # request without changing state; only the third actually changes it.
                    time.sleep(.3)
                    if len(mode_requests) >= 3:
                        state.mode = req.custom_mode
                    return SetModeResponse(len(mode_requests) >= 2)
                servers = [rospy.Service('/mavros/set_mode', SetMode, set_mode),
                           rospy.Service('/mavros/param/set', ParamSet, set_param),
                           rospy.Service('/mavros/param/get', ParamGet, get_param),
                           rospy.Service('/mavros/param/pull', ParamPull, pull)]
                subscribers = [rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget,
                                               attitude_command),
                               rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, position_command),
                               rospy.Subscriber('/pid_experiment/event', ExperimentEvent, events.append)]
                state = State(connected=True, armed=False, mode='POSCTL')
                pub = rospy.Publisher('/mavros/state', State, queue_size=1)
                thrust_pub = rospy.Publisher('/mavros/setpoint_raw/target_attitude', AttitudeTarget, queue_size=1)
                feedback_thrust = {'value': .338, 'enabled': True}
                motion = dict(x=0., z=3., vz=0., tilt=0., enabled=True, rate=0.)
                imu_pub = rospy.Publisher("/mavros/imu/data", Imu, queue_size=10)
                pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)
                vel_pub = rospy.Publisher('/mavros/local_position/velocity_local', TwistStamped, queue_size=1)
                sync_pub = rospy.Publisher('/mavros/timesync_status', TimesyncStatus, queue_size=1)
                def feedback(_):
                    pub.publish(state)
                    if motion['enabled']:
                        pose = PoseStamped()
                        pose.header.stamp = rospy.Time.now()
                        pose.pose.position.x = motion['x']; pose.pose.position.z = motion['z']
                        pose.pose.orientation.x = math.sin(math.radians(motion['tilt'])/2)
                        pose.pose.orientation.w = math.cos(math.radians(motion['tilt'])/2)
                        pose_pub.publish(pose)
                        vel = TwistStamped(); vel.header.stamp = rospy.Time.now()
                        vel.twist.linear.z = motion['vz']; vel_pub.publish(vel)
                        imu = Imu(); imu.header.stamp = rospy.Time.now()
                        imu.angular_velocity.x = motion['rate']; imu_pub.publish(imu)
                    if feedback_thrust['enabled']:
                        target = AttitudeTarget()
                        target.header.stamp = rospy.Time.now()
                        target.thrust = feedback_thrust['value']
                        thrust_pub.publish(target)
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
                values.update(config['baseline'])
                config['excitation'].update(settle_s=.1, pulse_duration_s=.08, repetitions=1)
                path = Path(tmp)/'config.yaml'; path.write_text(yaml.safe_dump(config))
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
                # Automatic flow: all candidates, low then high, confirmed POSCTL exit.
                def launch_auto(label):
                    state.mode = 'POSCTL'
                    state.armed = True
                    feedback_thrust.update(value=.34, enabled=True)
                    events.clear(); commands.clear(); mode_requests.clear()
                    position_commands.clear(); stream_times.clear()
                    motion.update(x=0., z=3., vz=0., tilt=0., enabled=True)
                    faults.update(write=False, mismatch=False, cache_ready=False)
                    path.write_text(yaml.safe_dump(config))
                    processes.append(subprocess.Popen(['roslaunch', 'holydrone_autotune', 'brct_autotune.launch',
                        'config:='+str(path), 'output:='+str(Path(tmp)/label)],
                        stdout=log, stderr=log, start_new_session=True))
                    rospy.wait_for_service('/get_experiment_status', timeout=15)
                    wait_for(lambda: 'state=READY' in status().message and 'thrust_ready=true' in status().message)
                    self.assertFalse(any(e.event_type == 'TRIAL_START' for e in events))

                def close_auto():
                    launch = processes.pop()
                    os.killpg(launch.pid, signal.SIGINT); launch.wait(timeout=15)

                # FCU stops thrust feedback before its delayed mode heartbeat arrives.
                for delay in (.8, 1.7):
                    launch_auto('handover_%s' % delay)
                    wait_for(lambda: len(commands) > 5)
                    feedback_thrust['enabled'] = False
                    start = time.monotonic()
                    time.sleep(delay)
                    if delay < 1.5:
                        self.assertLess(time.monotonic()-commands[-1][0], .2)
                    else:
                        self.assertGreater(time.monotonic()-commands[-1][0], .1)
                    state.mode = 'OFFBOARD'
                    if delay < 1.5:
                        wait_for(lambda: any(e.event_type == 'TRIAL_START' for e in events))
                        times = [t for t in stream_times if t >= start]
                        self.assertLess(max(b-a for a, b in zip(times, times[1:])), .2)
                        self.assertFalse(any(e.event_type == 'AUTO_RUN_REJECTED' for e in events))
                    else:
                        wait_for(lambda: 'POSCTL' in mode_requests)
                        self.assertTrue(any(e.event_type == 'AUTO_RUN_REJECTED' for e in events))
                        self.assertFalse(any(e.event_type == 'TRIAL_START' for e in events))
                    close_auto()

                launch_auto('auto_session')
                self.assertFalse(load('unknown').success)
                self.assertFalse(run('low').success)
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'TRIAL_START' for e in events))
                self.assertFalse(run('high').success)
                self.assertFalse(baseline().success)
                self.assertFalse(next_candidate().success)
                wait_for(lambda: 'state=COMPLETE' in status().message, timeout=30)
                starts = [(e.candidate_id, e.amplitude_deg_s) for e in events if e.event_type == 'TRIAL_START']
                expected = [(c['id'], level) for c in config['candidates']
                            for level in config['excitation']['levels_deg_s']]
                self.assertEqual(starts, expected)
                self.assertEqual(sum(e.event_type == 'TRIAL_FINISHED' for e in events), len(expected))
                self.assertEqual(mode_requests, ['POSCTL']*3)
                self.assertEqual(state.mode, 'POSCTL')
                self.assertTrue(any(e.event_type == 'SEQUENCE_COMPLETE' for e in events))
                self.assertEqual(values, {k: v for k, v in config['candidates'][-1].items() if k != 'id'})
                gaps = [b-a for a, b in zip(stream_times, stream_times[1:])]
                self.assertLess(max(gaps), .2)
                time.sleep(.15)
                state.mode = 'OFFBOARD'
                wait_for(lambda: 'offboard=true' in status().message)
                time.sleep(.2)
                self.assertEqual(sum(e.event_type == 'TRIAL_START' for e in events), len(expected))
                self.assertTrue(baseline().success)
                wait_for(lambda: 'state=READY' in status().message)
                self.assertEqual(values, config['baseline'])
                self.assertTrue(any(e.event_type == 'BASELINE_RESTORED' for e in events))
                close_auto()

                # Exit while applying the second candidate: late confirmation must not restart.
                launch_auto('exit_while_loading')
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'PARAMETER_APPLY_START' and
                                     e.candidate_id == config['candidates'][1]['id'] for e in events))
                state.mode = 'POSCTL'
                wait_for(lambda: any(e.event_type == 'SEQUENCE_ABORTED' for e in events))
                wait_for(lambda: 'state=READY' in status().message)
                time.sleep(.2)
                self.assertEqual(sum(e.event_type == 'TRIAL_START' for e in events), 2)
                self.assertFalse(mode_requests)
                # Re-entry restarts at the first candidate's low level.
                state.mode = 'OFFBOARD'
                wait_for(lambda: sum(e.event_type == 'TRIAL_START' for e in events) == 3)
                last = [e for e in events if e.event_type == 'TRIAL_START'][-1]
                self.assertEqual((last.candidate_id, last.amplitude_deg_s), expected[0])
                self.assertTrue(stop().success)
                time.sleep(.3)
                self.assertEqual(sum(e.event_type == 'TRIAL_START' for e in events), 3)
                close_auto()

                # Failed next-candidate readback stops automatic progress.
                launch_auto('failed_candidate')
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'PARAMETER_APPLY_START' and
                                     e.candidate_id == config['candidates'][1]['id'] for e in events))
                faults['mismatch'] = True
                wait_for(lambda: any(e.event_type == 'SEQUENCE_ABORTED' for e in events))
                self.assertTrue(any(e.event_type == 'PARAMETER_APPLY_FAILED' for e in events))
                time.sleep(.2)
                self.assertEqual(sum(e.event_type == 'TRIAL_START' for e in events), 2)
                self.assertFalse(mode_requests)
                close_auto()

                # An actual displacement must hold recovery; thrust grows during a tilted descent.
                config['excitation']['pulse_duration_s'] = .3
                launch_auto('recovery_gate')
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'PULSE_POSITIVE' for e in events))
                motion.update(x=.6, z=2.8, vz=-.2, tilt=30.)
                wait_for(lambda: commands[-1][1].thrust > .39)
                wait_for(lambda: any(e.event_type == 'RECOVERY_START' and e.phase == 2 for e in events))
                time.sleep(.4)
                self.assertEqual(sum(e.event_type in ('PULSE_POSITIVE', 'PULSE_NEGATIVE') for e in events), 1)
                self.assertFalse(position_commands)
                self.assertLess(commands[-1][1].body_rate.x, -.1)  # opposed to the injected roll tilt
                motion.update(x=1.2, z=3., vz=0., tilt=0.)
                time.sleep(.25)
                self.assertEqual(sum(e.event_type in ('PULSE_POSITIVE', 'PULSE_NEGATIVE') for e in events), 1)
                motion['x'] = .6  # bounded XY error is sufficient; exact origin is unnecessary
                wait_for(lambda: sum(e.event_type in ('PULSE_POSITIVE', 'PULSE_NEGATIVE') for e in events) == 2)
                # Former motion limits no longer trigger automatic mode switching.
                motion.update(z=1.8, x=6., vz=3., tilt=60.)
                time.sleep(.6)
                self.assertFalse(any(e.event_type == 'SAFETY_ABORT' for e in events))
                self.assertFalse(mode_requests)
                state.mode = 'POSCTL'
                wait_for(lambda: any(e.event_type == 'SEQUENCE_ABORTED' for e in events))
                close_auto()

                launch_auto('stale_feedback_manual_exit')
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'TRIAL_START' for e in events))
                motion['enabled'] = False
                time.sleep(.7)
                self.assertFalse(any(e.event_type == 'SAFETY_ABORT' for e in events))
                self.assertFalse(mode_requests)
                state.mode = 'POSCTL'
                wait_for(lambda: any(e.event_type == 'SEQUENCE_ABORTED' for e in events))
                close_auto()
                config['recovery']['timeout_s'] = .8
                launch_auto('oscillation_timeout')
                motion['rate'] = 3.0
                time.sleep(.1)
                state.mode = 'OFFBOARD'
                wait_for(lambda: any(e.event_type == 'SAFETY_ABORT' for e in events))
                self.assertFalse(any(e.event_type in ('PULSE_POSITIVE', 'PULSE_NEGATIVE') for e in events))
                self.assertTrue(any(e.event_type == 'SAFETY_ABORT' and 'recenter timeout' in e.detail for e in events))
                close_auto()
                timer.shutdown()
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
