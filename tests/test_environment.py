"""Offline regression checks for profiles and Webots prerequisite handling."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('webots_path', ROOT / 'scripts/webots_path.py')
webots_path = importlib.util.module_from_spec(spec)
spec.loader.exec_module(webots_path)


class EnvironmentTests(unittest.TestCase):
    def test_standard_driver_environment_precedence(self):
        try:
            import webots_ros2_driver.utils
        except ImportError:
            self.skipTest('ROS Webots driver unavailable; prerequisite check reports this separately')
        with tempfile.TemporaryDirectory() as temporary:
            homes = [Path(temporary) / name for name in ('primary', 'secondary')]
            for home in homes:
                (home / 'resources').mkdir(parents=True)
                (home / 'resources/version.txt').write_text('R2025a')
                (home / 'webots').touch(mode=0o755)
            with patch.dict(os.environ, {'ROS2_WEBOTS_HOME': str(homes[0]),
                                         'WEBOTS_HOME': str(homes[1])}):
                self.assertEqual(webots_path.resolve_webots()[0], homes[0] / 'webots')
                del os.environ['ROS2_WEBOTS_HOME']
                os.environ['WEBOTS_HOME'] = str(homes[1])
                self.assertEqual(webots_path.resolve_webots()[0], homes[1] / 'webots')

    def test_profiles_keep_navigation_out_of_basic(self):
        for profile, expected in [('basic', 2), ('navigation', 4), ('mapping', 7)]:
            process = subprocess.run(['bash', '-c',
                'source "$1/scripts/environment.sh"; ROOT=$1; simulation_profile "$2"; printf "%s\\n" "${SOURCE_PATHS[@]}"',
                'profile-test', str(ROOT), profile], capture_output=True, text=True, check=True)
            paths = process.stdout.splitlines()
            self.assertEqual(len(paths), expected)
            self.assertTrue(all((Path(path) / 'package.xml').is_file() for path in paths))
            if profile == 'basic':
                self.assertFalse(any('navigate' in path or 'third_party' in path for path in paths))

    def test_wrong_ros_distribution_fails_before_build(self):
        process = subprocess.run([str(ROOT / 'run_simulation.sh'), '--check-only'],
            env={**os.environ, 'ROS_DISTRO': 'humble'}, capture_output=True, text=True)
        self.assertEqual(process.returncode, 2)
        self.assertIn('Unsupported ROS_DISTRO', process.stderr)

    def test_webots_absence_and_release_checks(self):
        # Isolate repository behavior from installed driver discovery and graphics.
        class Version:
            def __init__(self, value):
                self.value = value
            def __str__(self):
                return self.value
            def __lt__(self, other):
                return self.value < other.value
            @staticmethod
            def from_path(path):
                return Version((Path(path) / 'resources/version.txt').read_text())
            @staticmethod
            def minimum():
                return Version('R2025a')
        utils = types.ModuleType('webots_ros2_driver.utils')
        utils.WebotsVersion = Version
        utils.get_webots_home = lambda: None
        with patch.dict(sys.modules, {'webots_ros2_driver.utils': utils}):
            with self.assertRaisesRegex(RuntimeError, 'Webots not found'):
                webots_path.resolve_webots()
            with tempfile.TemporaryDirectory() as temporary:
                home = Path(temporary)
                (home / 'resources').mkdir()
                (home / 'webots').touch(mode=0o755)
                utils.get_webots_home = lambda: temporary
                for version in ('R2023b', 'R2026a'):
                    (home / 'resources/version.txt').write_text(version)
                    with self.assertRaisesRegex(RuntimeError, 'Unsupported Webots'):
                        webots_path.resolve_webots()
                (home / 'resources/version.txt').write_text('R2025a')
                with patch.dict(os.environ):
                    self.assertEqual(webots_path.resolve_webots(), (home / 'webots', 'R2025a'))
                (home / 'webots').chmod(0o644)
                with self.assertRaisesRegex(RuntimeError, 'not executable'):
                    webots_path.resolve_webots()


if __name__ == '__main__':
    unittest.main()
