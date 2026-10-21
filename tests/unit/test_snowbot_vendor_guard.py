"""Offline Snowbot native guard checks; no ROS or hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from tools.deployment import prepare_snowbot_vendor as vendor
ROOT = Path(__file__).resolve().parents[2]

class VendorGuardTests(unittest.TestCase):
    def test_native_deadlines_and_hold(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp)/'guard')
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(vendor.ASSETS),
                            str(ROOT/'tests/native/test_snowbot_guard_118.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=10)
            self.assertIn('checks passed', result.stdout)
