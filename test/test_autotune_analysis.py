"""验证一阶合成脉冲的跟踪指标、不可测返回值及方向分组，无需 ROS Master。"""
import unittest
import numpy as np
from autotune_analysis import pulse_metrics, summarize


class AnalysisTest(unittest.TestCase):
    """使用合成角速度脉冲验证指标与汇总边界。"""
    def test_first_order_response(self):
        t = np.arange(0, .25, .001)
        y = 1-np.exp(-t/.02)
        rt = np.arange(.25, 1.25, .001)
        ry = np.exp(-(rt-.25)/.02)
        m = pulse_metrics(t, y, 1, rt, ry)
        self.assertAlmostEqual(m['rise_time_s'], .02*np.log(9), delta=.002)
        self.assertAlmostEqual(m['iae'], .02, delta=.001)
        self.assertEqual(m['overshoot_percent'], 0)
        self.assertAlmostEqual(m['return_time_s'], .02*np.log(20), delta=.002)
        self.assertGreater(m['return_rms'], 0)

    def test_unreached_and_unobserved(self):
        t = np.arange(0, .25, .01)
        m = pulse_metrics(t, np.zeros(len(t)), 1, [], [], hf_hz=60)
        self.assertIsNone(m['rise_time_s'])
        self.assertIsNone(m['return_time_s'])
        self.assertIsNone(m['return_rms'])
        self.assertIsNone(m['high_frequency_energy'])

    def test_repetitions_are_not_pooled_across_directions(self):
        rows = [dict(candidate_id='p1', amplitude_deg_s=60, direction=d, iae=v)
                for d, v in [('positive', 1.), ('positive', 3.), ('negative', 8.)]]
        groups = summarize(rows)
        self.assertEqual(groups[0]['iae']['median'], 2.)
        self.assertEqual(groups[0]['iae']['iqr'], 1.)
        self.assertEqual(groups[1]['iae']['median'], 8.)
