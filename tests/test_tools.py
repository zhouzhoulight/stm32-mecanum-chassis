"""不连接实物：验证串口选择与伪终端收发。"""
import importlib.util
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import time
import unittest
import json
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('project', ROOT/'scripts/project.py')
project = importlib.util.module_from_spec(spec)
spec.loader.exec_module(project)


class ToolsTest(unittest.TestCase):
    def test_pyocd_0451_real_boards_response(self):
        # 来自本次实际运行 pyocd json --probes 的输出结构。
        response = {'pyocd_version':'0.45.1', 'version':{'major':1,'minor':1},
                    'status':0, 'boards':[{'unique_id':'CMSIS-DAP',
                    'info':'CMSIS-DAP STM32  CMSIS-DAP', 'target':'cortex_m',
                    'vendor_name':'CMSIS-DAP', 'product_name':'STM32  CMSIS-DAP'}]}
        with patch.object(project, 'pyocd_tool', return_value=Path('/fake-tool')), \
             patch.object(project.subprocess, 'check_output', return_value=json.dumps(response)):
            self.assertEqual(project.probe_list()[0]['unique_id'], 'CMSIS-DAP')
            self.assertEqual(project.choose_probe('auto'), 'CMSIS-DAP')

    def test_pyocd_empty_boards_and_alternate_probes_field(self):
        with patch.object(project, 'pyocd_tool', return_value=Path('/fake-tool')), \
             patch.object(project.subprocess, 'check_output') as output:
            output.return_value=json.dumps({'status':0,'boards':[]})
            self.assertEqual(project.probe_list(), [])
            output.return_value=json.dumps({'status':0,'probes':[{'unique_id':'OTHER'}]})
            self.assertEqual(project.probe_list()[0]['unique_id'], 'OTHER')
            output.return_value=json.dumps({'status':0,'unexpected':[]})
            with self.assertRaises(SystemExit):
                project.probe_list()

    def test_port_selection_does_not_guess(self):
        with patch.object(project, 'ports', return_value=[]):
            with self.assertRaises(SystemExit):
                project.choose_port('auto')
        with patch.object(project, 'ports', return_value=['/dev/cu.usb1', '/dev/cu.usb2']):
            with self.assertRaises(SystemExit):
                project.choose_port('auto')
            self.assertEqual(project.choose_port('/dev/cu.usb2'), '/dev/cu.usb2')

    def test_build_failure_prevents_flash(self):
        with patch.object(project, 'choose_probe', return_value='FAKE-DAP'), \
             patch.object(project, 'pyocd_tool', return_value=Path('/fake-tool')), \
             patch.object(project, 'build', side_effect=subprocess.CalledProcessError(1, 'build')), \
             patch.object(project.subprocess, 'run') as run:
            with self.assertRaises(subprocess.CalledProcessError):
                project.flash('auto')
            run.assert_not_called()

    def test_probe_selection_does_not_guess(self):
        with patch.object(project, 'probe_list', return_value=[]):
            with self.assertRaises(SystemExit):
                project.choose_probe('auto')
        with patch.object(project, 'probe_list', return_value=[{'unique_id':'a'}, {'unique_id':'b'}]):
            with self.assertRaises(SystemExit):
                project.choose_probe('auto')
            self.assertEqual(project.choose_probe('b'), 'b')

    def test_flash_uses_exact_run_config_without_chip_erase_or_reset(self):
        with patch.object(project, 'choose_probe', return_value='FAKE-DAP'), \
             patch.object(project, 'pyocd_tool', return_value=Path('/fake-tool')), \
             patch.object(project, 'build'), \
             patch('builtins.input', return_value='FLASH'), \
             patch.object(project.subprocess, 'run') as run:
            project.flash('auto')
            cmd=run.call_args.args[0]
            self.assertEqual(cmd[cmd.index('--probe')+1], 'cmsisdap:FAKE-DAP')
            self.assertEqual(cmd[cmd.index('--erase')+1], 'sector')
            self.assertEqual(cmd[cmd.index('--cbuild-run')+1], str(project.RUN_FILE))
            self.assertIn('--no-reset', cmd)
            self.assertIn('--no-wait', cmd)

    def test_console_on_pseudo_terminal(self):
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)
        code = ('import sys;sys.path.insert(0,sys.argv[1]);import project;'
                'project.choose_port=lambda _:sys.argv[2];project.console("fake")')
        proc = subprocess.Popen([sys.executable, '-u', '-c', code, str(ROOT/'scripts'), slave_name],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        def receive_until(token):
            data = b''
            deadline = time.monotonic() + 5
            while token not in data and time.monotonic() < deadline:
                if select.select([master], [], [], 0.1)[0]:
                    data += os.read(master, 4096)
            self.assertIn(token, data)
            return data

        try:
            self.assertTrue(select.select([proc.stdout], [], [], 5)[0])
            self.assertIn('已打开', proc.stdout.readline().decode())
            self.assertIn('help', proc.stdout.readline().decode())
            self.assertEqual(receive_until(b'\r\n'), b'\r\n') # 打开只发空行同步，不发动作。
            proc.stdin.write(b'help\n'); proc.stdin.flush()
            self.assertEqual(receive_until(b'help\r\n'), b'help\r\n')
            for command in (b'arm all', b'forward', b'back'):
                proc.stdin.write(command+b'\n'); proc.stdin.flush()
                self.assertEqual(receive_until(command+b'\r\n'), command+b'\r\n')
            os.write(master, b'BOARD_READY\r\n')
            proc.stdin.close(); proc.stdin = None
            # macOS 的伪终端必须先读走输出，子进程的 tcdrain 才会完成。
            self.assertEqual(receive_until(b'stop\r\n'), b'stop\r\n')
            output, errors = proc.communicate(timeout=5)
            self.assertEqual(proc.returncode, 0, errors)
            self.assertIn(b'BOARD_READY', output)
        finally:
            if proc.poll() is None:
                proc.kill(); proc.communicate()
            os.close(master); os.close(slave)


if __name__ == '__main__':
    unittest.main(verbosity=2)
