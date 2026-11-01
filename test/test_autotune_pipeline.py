"""Synthetic real .ulg and time synchronization files exercise clock joining and complete report generation."""
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
    def test_ulog_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            times = np.arange(0, 6, .005)
            offset = 1700000000.
            segments = [(.75, 0), (.25, 1), (.25, -1), (.75, 0), (.25, -1), (.25, 1), (1., 0)]
            start = 1.
            events = [dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1', event_type='TRIAL_START')]
            sp = np.zeros(len(times))
            for phase, (duration, sign) in enumerate(segments):
                events.append(dict(timestamp=offset+start, trial_id=1, axis='roll', candidate_id='p1',
                                   event_type='PULSE_ZERO' if sign == 0 else 'PULSE_POSITIVE' if sign > 0 else 'PULSE_NEGATIVE',
                                   amplitude_deg_s=60, repetition=0, phase=phase))
                sp[(times >= start) & (times < start+duration)] = sign*np.deg2rad(60)
                start += duration
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
            script = Path(__file__).resolve().parents[1]/'scripts/brct_autotune_analyze.py'
            command = [sys.executable, str(script), '--session', str(root), '--ulog', str(root/'flight.ulg'),
                       '--output', str(root/'report')]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads((root/'report/report.json').read_text())
            self.assertEqual(len(report['pulses']), 4)
            self.assertEqual(len(report['summary']), 2)
            self.assertTrue((root/'report/trial_001.png').exists())
            self.assertAlmostEqual(report['pulses'][0]['allocator_unachieved_s'], .25, places=5)
            # A materially wrong clock offset must be rejected, not silently analyzed.
            command[-1] = str(root/'wrong_report')
            result = subprocess.run(command+['--ros-minus-px4-s', str(offset+.3)],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('setpoint mismatch', result.stderr)
