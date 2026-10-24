import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/prepare_build.py'
spec = importlib.util.spec_from_file_location('prepare_build', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PrepareBuildTest(unittest.TestCase):
    def test_real_project_metadata_and_version_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'project'
            calls = []

            def conduct(command, **kwargs):
                calls.append(command)
                if 'new-project' in command:
                    (destination / 'src').mkdir(parents=True)
                    (destination / 'include/lemlib').mkdir(parents=True)
                    (destination / 'common.mk').touch()
                    (destination / 'include/lemlib/api.hpp').touch()
                    (destination / 'project.pros').write_text(json.dumps({
                        'py/state': {'templates': {'kernel': {'version': '4.2.1'},
                                      'LemLib': {'version': '0.5.6'}}}}))

            with patch.object(module.shutil, 'which', return_value='/bin/pros'), \
                 patch.object(module.subprocess, 'run', side_effect=conduct), \
                 patch('sys.argv', ['prepare_build', '--kernel-version', '4.2.1',
                                   '--lemlib-version', '0.5.6', str(destination)]):
                module.main()
            self.assertIn('LemLib@0.5.6', calls[-1])
            self.assertEqual(json.loads((destination / 'dependency-versions.json').read_text())['kernel'], '4.2.1')
            self.assertIn('506', (destination / 'include/autotune/dependency_version.hpp').read_text())
            self.assertTrue((destination / 'src/autotune/lemlib_tuning_drive.cpp').is_file())
            self.assertTrue((destination / 'docs/compatibility.md').is_file())

    def test_existing_destination_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'keep'
            marker.write_text('unchanged')
            with patch('sys.argv', ['prepare_build', directory]), self.assertRaises(SystemExit):
                module.main()
            self.assertEqual(marker.read_text(), 'unchanged')


if __name__ == '__main__':
    unittest.main()
