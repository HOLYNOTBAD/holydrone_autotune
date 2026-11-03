"""验证重复同步采样、SITL 共享时钟校验及实飞/时钟重置拒绝。"""
import copy
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('autotune_analyzer',
    Path(__file__).resolve().parents[1]/'scripts/autotune_analyze.py')
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


class ClockAlignment(unittest.TestCase):
    def setUp(self):
        self.t = np.arange(0, 6, .01)
        self.sp = dict(roll=np.zeros(len(self.t)))
        self.events = [dict(timestamp=.8, trial_id=1, event_type='TRIAL_START')]
        for start, sign in [(1., 1), (1.25, -1), (2., -1), (2.25, 1)]:
            self.sp['roll'][(self.t >= start) & (self.t < start+.25)] = sign*np.pi/3
            self.events.append(dict(timestamp=start, trial_id=1,
                                    event_type='PULSE_POSITIVE' if sign > 0 else 'PULSE_NEGATIVE',
                                    axis='roll', amplitude_deg_s=60))
            self.events.append(dict(timestamp=start+.25, trial_id=1, event_type='PULSE_FINISHED'))
        self.events.append(dict(timestamp=3., trial_id=1, event_type='TRIAL_FINISHED'))
        self.sync = np.column_stack([np.arange(0, 6, .1), np.zeros(60), np.zeros(60)])

    def resolve(self, sync, info=None, events=None, override=None, sp=None):
        return analyzer.resolve_clock(self.events if events is None else events, sync, override,
                                      info or {}, self.t, self.sp if sp is None else sp, [self.t]*4)

    def test_passthrough_header_only_timesync_still_requires_pulse_validation(self):
        _, info = self.resolve([], dict(ver_hw='PX4_SITL'))
        self.assertEqual(info['validated_pulses'], 4)
        with self.assertRaisesRegex(ValueError, 'restricted to validated PX4_SITL'):
            self.resolve([], dict(ver_hw='PX4_FMU_V6X'))

    def test_duplicate_ros_samples_are_not_a_clock_reset(self):
        sync = np.insert(self.sync, 10, self.sync[10], axis=0)
        convert, info = self.resolve(sync)
        self.assertEqual(info['source'], 'timesync')
        self.assertEqual(info['duplicate_samples'], 1)
        self.assertEqual(info['validated_pulses'], 4)
        self.assertEqual(convert(self.events[0]), .8)

    def test_valid_nonzero_timesync_is_preferred(self):
        events = copy.deepcopy(self.events)
        for event in events: event['timestamp'] += 1000
        sync = self.sync.copy(); sync[:, 0] += 1000; sync[:, 1] = 1000
        convert, info = self.resolve(sync, dict(ver_hw='PX4_SITL'), events)
        self.assertEqual(info['source'], 'timesync')
        self.assertAlmostEqual(convert(events[0]), .8)

    def test_bad_sitl_timesync_falls_back_only_after_all_pulses_match(self):
        sync = self.sync.copy(); sync[:, 1] = 1e9
        convert, info = self.resolve(sync, dict(ver_hw='PX4_SITL'))
        self.assertEqual(info['source'], 'sitl_shared_clock_validated')
        self.assertEqual(info['validated_pulses'], 4)
        self.assertEqual(info['max_median_setpoint_error_rad_s'], 0)
        self.assertLess(max(abs(x) for x in info['observed_edge_delay_range_s']), .011)
        self.assertEqual(convert(self.events[0]), .8)

    def test_bad_real_flight_sync_does_not_assume_zero(self):
        sync = self.sync.copy(); sync[:, 1] = 1e9
        with self.assertRaisesRegex(ValueError, 'restricted to validated PX4_SITL'):
            self.resolve(sync, dict(ver_hw='PX4_FMU_V6X'))

    def test_actual_clock_reset_is_rejected(self):
        sync = self.sync.copy(); sync[10, 0] = .5
        with self.assertRaisesRegex(ValueError, 'moved backwards'):
            self.resolve(sync, dict(ver_hw='PX4_SITL'))
        events = copy.deepcopy(self.events); events[-1]['timestamp'] = .2
        with self.assertRaisesRegex(ValueError, 'event clock moved backwards'):
            self.resolve(self.sync, dict(ver_hw='PX4_SITL'), events)
        with self.assertRaisesRegex(ValueError, 'event clock moved backwards'):
            self.resolve([], dict(ver_hw='PX4_SITL'), events, override=0)

    def test_wrong_sitl_log_fails_shared_clock_validation(self):
        sync = self.sync.copy(); sync[:, 1] = 1e9
        with self.assertRaisesRegex(ValueError, 'shared clock validation failed'):
            self.resolve(sync, dict(ver_hw='PX4_SITL'), sp=dict(roll=np.zeros(len(self.t))))

    def test_insufficient_pulses_do_not_establish_shared_clock(self):
        sync = self.sync.copy(); sync[:, 1] = 1e9
        events = self.events[:5]+[self.events[-1]]
        with self.assertRaisesRegex(ValueError, 'insufficient positive/negative pulses'):
            self.resolve(sync, dict(ver_hw='PX4_SITL'), events)

    def test_explicit_offset_remains_available(self):
        convert, info = self.resolve([], override=123)
        self.assertEqual(info['source'], 'explicit')
        self.assertAlmostEqual(convert(self.events[0]), .8-123)
