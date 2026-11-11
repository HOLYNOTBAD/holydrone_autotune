"""验证无参数选择最新会话，以及会话内唯一 ULog 的约束。"""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('analyze_inputs',
    Path(__file__).resolve().parents[1]/'scripts/autotune_analyze.py')
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


class Inputs(unittest.TestCase):
    def test_default_session_and_ulog_requirements(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            older, latest = [root/'data'/name for name in ('old', 'latest with space')]
            for stamp, session in enumerate((older, latest), 1):
                session.mkdir(parents=True)
                event = session/'events.yaml'
                event.write_text('')
                os.utime(event, (stamp, stamp))
            (older/'old.ulg').touch()
            sibling = latest.with_name(latest.name+'_px4')
            sibling.mkdir()
            (sibling/'flight.ulg').touch()
            # Ignore runtime/result directories even if they contain copied events.
            (sibling/'events.yaml').touch()
            with patch.object(analyzer, 'PACKAGE', root), patch('sys.argv', ['analyze']), \
                    patch.object(analyzer, 'analyze') as run:
                with self.assertRaises(SystemExit):
                    analyzer.main()  # no fallback to older session or sibling _px4
                run.assert_not_called()
                (latest/'flight.ulg').touch()
                (latest/'second.ulg').touch()
                with self.assertRaises(SystemExit):
                    analyzer.main()
                run.assert_not_called()
                (latest/'second.ulg').unlink()
                analyzer.main()
                args, selected, output = run.call_args.args
                self.assertEqual(selected, latest)
                self.assertEqual(args.ulog, latest/'flight.ulg')
                self.assertTrue(latest.with_name(latest.name+'_result').is_dir())
                with self.assertRaises(SystemExit):
                    analyzer.main()  # preserve existing results


if __name__ == '__main__':
    unittest.main()
