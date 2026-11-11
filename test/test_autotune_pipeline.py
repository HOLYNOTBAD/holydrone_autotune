"""生成合成 ULog、事件与时间同步文件，验证离线报告流水线；无需飞控或仿真。"""
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import json
import numpy as np
import yaml


class PipelineTest(unittest.TestCase):
    """在临时目录生成 ULog 与会话文件，并调用真实离线分析入口。"""
    def test_ulog_report(self):
        self.run_report(False)

    def test_recovery_after_each_pulse(self):
        self.run_report(True)

    def test_contiguous_four_pulse_group(self):
        self.run_report(True, grouped=True)

    def run_report(self, each_pulse, grouped=False):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            times = np.arange(0, 6, .005)
            offset = 1700000000.
            segments = [(.75, 0), (.25, 1), (.25, -1), (.75, 0), (.25, -1), (.25, 1), (1., 0)]
            if each_pulse:
                segments = [(.3, 0), (.25, 1), (.3, 0), (.25, -1), (.3, 0), (.25, -1), (.3, 0), (.25, 1), (.3, 0)]
            if grouped:
                segments = [(.3, 0), (.25, 1), (.25, -1), (.25, -1), (.25, 1), (.3, 0)]
            start = 1.
            events = [dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1', amplitude_deg_s=60, event_type='TRIAL_START')]
            sp = np.zeros(len(times))
            for phase, (duration, sign) in enumerate(segments):
                events.append(dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1',
                                   event_type=('RECOVERY_START' if each_pulse else 'PULSE_ZERO') if sign == 0 else 'PULSE_POSITIVE' if sign > 0 else 'PULSE_NEGATIVE',
                                   amplitude_deg_s=60, repetition=0, phase=phase))
                sp[(times >= start) & (times < start+duration)] = sign*np.deg2rad(60)
                start += duration
                if each_pulse:
                    events.append(dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1',
                                       event_type='PULSE_FINISHED' if sign else 'RECOVERY_FINISHED',
                                       amplitude_deg_s=60, repetition=0, phase=phase))
            events.append(dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1', event_type='TRIAL_FINISHED'))
            (root/'events.yaml').write_text(yaml.safe_dump_all(events))
            with (root/'timesync.csv').open('w') as stream:
                stream.write('ros_s,ros_minus_px4_s,round_trip_time_ms\n')
                for t in np.arange(0, 6, .1):
                    stream.write('%.9f,%.9f,1.0\n' % (offset+t, offset))
            av = np.zeros(len(times))
            for i in range(1, len(times)):
                av[i] = av[i-1]+.2*(sp[i]-av[i-1])
            topics = [('vehicle_angular_velocity', 'float[3] xyz;', av),
                      ('vehicle_rates_setpoint', 'float roll;float pitch;float yaw;', sp),
                      ('vehicle_torque_setpoint', 'float[3] xyz;', .1*(sp-av)),
                      ('control_allocator_status', 'float[3] unallocated_torque;', np.ones(len(times))*.01)]
            topics += [(name, 'float[3] xyz;', np.zeros(len(times))) for name in
                       ['vehicle_attitude', 'vehicle_local_position', 'battery_status', 'vehicle_status']]
            with (root/'flight.ulg').open('wb') as f:
                f.write(b'ULog\x01\x12\x35\x01'+struct.pack('<Q', 0))
                def record(kind, payload):
                    f.write(struct.pack('<HB', len(payload), ord(kind))+payload)
                for mid, (name, fields, data) in enumerate(topics):
                    record('F', (name+':uint64_t timestamp;'+fields).encode())
                for mid, (name, fields, data) in enumerate(topics):
                    record('A', struct.pack('<BH', 0, mid)+name.encode())
                    for t, value in zip(times, data):
                        record('D', struct.pack('<HQfff', mid, int(round(t*1e6)), value, 0, 0))
                    if name == 'vehicle_status':
                        # 同一PX4时刻可以发布两次状态更新，不应把它当成时钟倒退。
                        record('D', struct.pack('<HQfff', mid, int(round(times[-1]*1e6)), 0, 0, 0))
            script = Path(__file__).resolve().parents[1]/'scripts/autotune_analyze.py'
            command = [sys.executable, str(script), '--session', str(root),
                       '--output', str(root/'report')]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads((root/'report/report.json').read_text())
            self.assertEqual(len(report['pulses']), 4)
            self.assertEqual(len(report['summary']), 2)
            self.assertTrue((root/'report/comparison_roll_60deg_s.png').exists())
            self.assertTrue((root/'report/comparison_roll_60deg_s_zoom.png').exists())
            self.assertTrue((root/'report/summary.csv').exists())
            self.assertGreater(report['pulses'][0]['tracking_rmse'], 0)
            self.assertLess(report['pulses'][0]['tracking_rmse'], np.deg2rad(60))
            # A materially wrong clock offset must be rejected, not silently analyzed.
            command[-1] = str(root/'wrong_report')
            result = subprocess.run(command+['--ros-minus-px4-s', str(offset+.3)],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('setpoint mismatch', result.stderr)
