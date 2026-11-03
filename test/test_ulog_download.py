"""离线验证落地触发、日志匹配和 FTP 文件完整性；不连接飞控。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('download_flight_ulog',
    Path(__file__).resolve().parents[1]/'scripts/download_flight_ulog.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DownloadTests(unittest.TestCase):
    def test_only_landed_flight_disarm_triggers(self):
        m = module.FlightMonitor()
        m.state(True, False, 0)
        m.extended(1, 0)
        self.assertIsNone(m.pending)
        m.baseline = {'old.ulg': 123}
        m.state(True, True, 1)
        m.state(True, False, 2)  # 地面试解锁不能触发。
        self.assertIsNone(m.pending)
        m.baseline = {'old.ulg': 123}
        m.state(True, True, 3)
        m.extended(2, 3)
        m.state(False, False, 4)  # 断连不能误当成 disarm。
        self.assertIsNone(m.pending)
        m.state(True, False, 5)
        self.assertEqual(m.pending['baseline'], {'old.ulg': 123})
        self.assertFalse(m.safe(5))  # 仍须确认落地。
        m.extended(1, 5)
        self.assertTrue(m.safe(5))
        self.assertFalse(m.safe(9))
        m.state(True, True, 6)
        self.assertIsNone(m.pending)
        self.assertFalse(m.safe(6))

    def fixture(self, root, data, check=lambda: None):
        closed = []
        def read(file_path, offset, size):
            # 短读和分包不能截断结果。
            return SimpleNamespace(success=True, r_errno=0, data=data[offset:offset+min(size, 137)])
        ftp = dict(open=lambda **kw: SimpleNamespace(success=True, size=len(data)), read=read,
                   close=lambda **kw: closed.append(kw['file_path']))
        return module.Downloader(ftp, '/fs/microsd/log', root, check, lambda *args: None), closed

    def test_complete_file_and_retry(self):
        data = b'ULog\x01\x12\x35'+bytes(range(256))*100
        with tempfile.TemporaryDirectory() as tmp:
            d, closed = self.fixture(tmp, data)
            path = d.download('/fs/microsd/log/2026-11-02/flight.ulg', len(data), 1)
            self.assertEqual(path.read_bytes(), data)
            self.assertFalse(path.with_suffix('.ulg.part').exists())
            self.assertEqual(d.download('/fs/microsd/log/2026-11-02/flight.ulg', len(data), 1), path)
            self.assertEqual(len(closed), 1)

    def test_rearm_aborts_and_preserves_partial(self):
        data = b'ULog\x01\x12\x35'+b'x'*2000
        with tempfile.TemporaryDirectory() as tmp:
            calls = [0]
            def check():
                calls[0] += 1
                if calls[0] > 6:
                    raise RuntimeError('armed again')
            d, closed = self.fixture(tmp, data, check)
            with self.assertRaisesRegex(RuntimeError, 'armed again'):
                d.download('/fs/microsd/log/flight.ulg', len(data), 1)
            self.assertFalse(list(Path(tmp).rglob('*.ulg')))
            self.assertEqual(len(list(Path(tmp).rglob('*.part'))), 1)
            self.assertEqual(len(closed), 1)

    def test_invalid_or_growing_log_not_completed(self):
        with tempfile.TemporaryDirectory() as tmp:
            d, closed = self.fixture(tmp, b'invalid data'*100)
            with self.assertRaisesRegex(RuntimeError, 'not a ULog'):
                d.download('/fs/microsd/log/bad.ulg', 1200, 1)
            with self.assertRaisesRegex(RuntimeError, 'size still changing'):
                d.download('/fs/microsd/log/growing.ulg', 1300, 1)
            self.assertFalse(list(Path(tmp).rglob('*.ulg')))
            self.assertEqual(len(closed), 2)

    def test_inventory_and_new_sd_card(self):
        with tempfile.TemporaryDirectory() as tmp:
            d, _ = self.fixture(tmp, b'')
            def listing(dir_path):
                entries = ([SimpleNamespace(name='2026-11-02', type=1)] if dir_path.endswith('/log')
                           else [SimpleNamespace(name='flight.ulg', type=0, size=100),
                                 SimpleNamespace(name='note.txt', type=0, size=20)])
                return SimpleNamespace(success=True, list=entries)
            d.ftp['list'] = listing
            self.assertEqual(d.inventory(), {'/fs/microsd/log/2026-11-02/flight.ulg': 100})
            d.ftp['list'] = lambda **kw: SimpleNamespace(success=False, r_errno=2)
            self.assertEqual(d.inventory(), {})


if __name__ == '__main__':
    unittest.main()
