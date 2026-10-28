#!/usr/bin/env python3
"""Run host tests; optionally compile/inspect firmware. Never connects to a board."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', action='store_true', help='also build and inspect the ARM ELF')
    args = parser.parse_args()
    out = ROOT/'build/host-tests'
    out.mkdir(parents=True, exist_ok=True)
    pairs = [('motor', 'motor_test'), ('imu', 'wit_imu'), ('yaw_trace', 'yaw_trace'),
             ('heading', 'heading_hold')]
    for test, module in pairs:
        if not (ROOT/f'tests/test_{test}.c').exists():
            continue
        exe = out/f'test_{test}'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-Iinclude', f'tests/test_{test}.c',
                        f'src/{module}.c', '-o', str(exe)], cwd=ROOT, check=True)
        subprocess.run([str(exe)], cwd=ROOT, check=True)
    for name in ['test_tools.py', 'test_driver_parameters.py']:
        if (ROOT/'tests'/name).exists():
            subprocess.run([sys.executable, str(ROOT/'tests'/name)], cwd=ROOT, check=True)
    if args.firmware:
        subprocess.run([sys.executable, str(ROOT/'scripts/project.py'), 'build'], cwd=ROOT, check=True)
        subprocess.run([sys.executable, str(ROOT/'tests/verify_firmware.py')], cwd=ROOT, check=True)
    print('PASS: requested checks finished; no board, serial port or motor was accessed.')


if __name__ == '__main__':
    main()
