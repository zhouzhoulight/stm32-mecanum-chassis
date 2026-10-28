#!/usr/bin/env python3
"""F407：Keil 编译、CMSIS-DAP 下载、USART1 控制台。编译不会自动烧录。"""
import argparse
import glob
import os
from pathlib import Path
import shutil
import subprocess
import sys
import json

ROOT = Path(__file__).resolve().parents[1]
IDE = Path('/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins')
RUN_FILE = ROOT/'out/chassis+Board.cbuild-run.yml'
FIRMWARE = ROOT/'build/chassis-f407-test.elf'


def find_tool(env_name, pattern, executable):
    override = os.environ.get(env_name)
    if override:
        candidate = Path(override)
        if candidate.is_file():
            return candidate
        raise SystemExit(f'{env_name} 指定的工具不存在：{candidate}')
    candidates = sorted(glob.glob(pattern))
    if candidates:
        return Path(candidates[-1])
    found = shutil.which(executable)
    if found:
        return Path(found)
    raise SystemExit(f'找不到 {executable}。可用环境变量 {env_name} 指定可执行文件。')


def build_environment():
    gcc = find_tool('CHASSIS_GCC', str(IDE/'*gnu-tools*/tools/bin/arm-none-eabi-gcc'), 'arm-none-eabi-gcc')
    cbuild = find_tool('CHASSIS_CBUILD', str(Path.home()/'.vscode/extensions/arm.cmsis-csolution-*/tools/cmsis-toolbox/bin/cbuild'), 'cbuild')
    env = os.environ.copy()
    version = subprocess.check_output([str(gcc), '-dumpfullversion'], text=True).strip()
    env['GCC_TOOLCHAIN_' + version.replace('.', '_')] = str(gcc.parent)
    env['PATH'] = os.pathsep.join([str(cbuild.parent), str(gcc.parent), '/opt/homebrew/bin', env.get('PATH', '')])
    return cbuild, gcc, env


def build():
    cbuild, gcc, env = build_environment()
    subprocess.run([str(cbuild), 'chassis.csolution.yml', '--active', 'Board', '--update-rte'],
                   cwd=ROOT, env=env, check=True)
    output = ROOT/'out/chassis/Board/Debug'
    (ROOT/'build').mkdir(exist_ok=True)
    for suffix in ('elf', 'hex', 'bin'):
        source = output/f'chassis.{suffix}'
        if not source.is_file():
            raise SystemExit(f'编译输出不存在：{source}，停止，不使用旧固件。')
        shutil.copy2(source, ROOT/'build'/f'chassis-f407-test.{suffix}')
    if not RUN_FILE.is_file():
        raise SystemExit('缺少 CMSIS 下载配置，停止，不使用默认芯片型号。')
    subprocess.run([str(gcc.with_name('arm-none-eabi-size')), str(FIRMWARE)], check=True)
    print('\n编译成功：build/chassis-f407-test.hex；没有下载或启动电机。')


def ports():
    # 不碰蓝牙和调试控制台，只列 USB 串口候选；多个端口时不猜测。
    return sorted(p for p in glob.glob('/dev/cu.*')
                  if any(t in p.lower() for t in ('usb', 'wch', 'slab')))


def choose_port(value):
    if value and value != 'auto':
        if value not in ports():
            raise SystemExit(f'端口不在当前 USB 串口列表中：{value}；列表：{ports()}')
        return value
    available = ports()
    if len(available) != 1:
        raise SystemExit(f'需要恰好一个 USB 串口，当前列表：{available}。插入适配器，或用 --port 指定。')
    return available[0]


def pyocd_tool():
    return find_tool('CHASSIS_PYOCD', str(Path.home()/'.vscode/extensions/arm.vscode-cmsis-debugger-*/tools/pyocd/pyocd'), 'pyocd')


def probe_list():
    data = json.loads(subprocess.check_output([str(pyocd_tool()), 'json', '--probes'], text=True))
    if data.get('status', 0) != 0:
        raise SystemExit(f'读取调试器失败：{data}')
    # pyOCD 0.45.1 的 --probes JSON 仍使用历史字段 boards。
    # 不能把缺少 probes 字段误判成电脑没有连接下载器。
    if 'boards' in data:
        available = data['boards']
    elif 'probes' in data:
        available = data['probes']
    else:
        raise SystemExit(f'无法识别 pyOCD 的枚举格式，字段为 {list(data)}，不能据此判断设备未连接。')
    if not isinstance(available, list) or any(
        not isinstance(item, dict) or not isinstance(item.get('unique_id'), str)
        for item in available
    ):
        raise SystemExit('pyOCD 返回的调试器列表格式异常。')
    return available


def choose_probe(value):
    available = probe_list()
    if value and value != 'auto':
        matches = [p for p in available if p.get('unique_id') == value]
        if not matches:
            raise SystemExit('指定的调试器未连接。先运行 probes 查看完整序列号。')
        return value
    if len(available) != 1:
        raise SystemExit(f'需要恰好一个 CMSIS-DAP，当前可见调试器数量：{len(available)}。先运行 probes；多个时用 --probe 指定。')
    return available[0]['unique_id']


def flash(probe, confirm=True):
    uid = choose_probe(probe)
    tool = pyocd_tool()
    build()  # 编译失败就终止，不能误烧旧固件。
    print('目标 STM32F407VET6 / 512 KiB；CMSIS-DAP SWD 1 MHz。BOOT0 保持 0。')
    print('请保持电机断电。下载会替换新板内的程序，不会自动备份。')
    if confirm and input('输入 FLASH 确认下载，其他输入取消：').strip() != 'FLASH':
        raise SystemExit('已取消，没有下载。')
    subprocess.run([str(tool), 'load', '--probe', f'cmsisdap:{uid}', '--no-wait',
                    '--cbuild-run', str(RUN_FILE), '--frequency', '1000000',
                    '--erase', 'sector', '--no-reset', '-O', 'resume_on_disconnect=false'],
                   cwd=ROOT, check=True)
    print('CMSIS-DAP 写入完成，目标保持停止。按主板 RESET 运行；调试时由 F5 连接后继续运行。')


def console(port):
    # macOS 原生串口接口，不需要另外安装 Python 包。
    import select
    import termios
    port = choose_port(port)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    config = termios.tcgetattr(fd)
    config[0] = 0
    config[1] = 0
    config[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    config[3] = 0
    config[4] = config[5] = termios.B115200
    config[6][termios.VMIN] = 0
    config[6][termios.VTIME] = 0
    allowed = {'help', 'status', 'imu', 'stop', 'jog 0', 'jog 1', 'arm all',
               'forward', 'back', 'left', 'right', 'cw', 'ccw',
               'forward_long', 'back_long', 'left_long', 'right_long', 'cw_long', 'ccw_long',
               'forward_16000', 'forward_1m', 'back_1m', 'left_16000', 'right_16000'} | {f'arm {i}' for i in range(1, 5)} | {f'probe {i}' for i in range(1, 5)} | {f'imu baud {baud}' for baud in (4800,9600,19200,38400,57600,115200,230400,460800,921600)}

    def send(data):
        while data:
            _, ready, _ = select.select([], [fd], [], 1)
            if not ready:
                raise OSError('串口写入超时')
            n = os.write(fd, data)
            data = data[n:]

    try:
        termios.tcsetattr(fd, termios.TCSANOW, config)
        termios.tcflush(fd, termios.TCIOFLUSH)
        # 先结束主控上可能残留的半条命令；空行不会使能或启动电机。
        send(b'\r\n')
        print(f'已打开 {port}，115200 8N1。USART1：模块 TX→PA10，RX←PA9，GND 共地。')
        print('先输入 status 或 help，收到板子回复后再测试。Ctrl+C 退出并尝试发送 stop。')
        while True:
            ready, _, _ = select.select([fd, sys.stdin], [], [], 0.2)
            if fd in ready:
                data = os.read(fd, 4096)
                if data:
                    print(data.decode('ascii', errors='replace'), end='', flush=True)
            if sys.stdin in ready:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
                if line not in allowed:
                    print('可用命令：status、help、imu、imu baud N、probe 1～4、arm 1～4、jog 0/1、arm all、六种方向及_long、forward_16000/left_16000/right_16000、forward_1m/back_1m、stop。')
                    continue
                send((line+'\r\n').encode('ascii'))
    except KeyboardInterrupt:
        print('\n退出串口控制台。')
    finally:
        try:
            send(b'stop\r\n')
            termios.tcdrain(fd)
        except OSError:
            print('串口已断开，stop 未能发出。')
        try:
            termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['build', 'ports', 'probes', 'flash', 'debug-load', 'console'])
    parser.add_argument('--port', default='auto')
    parser.add_argument('--probe', default='auto')
    args = parser.parse_args()
    if args.action == 'build':
        build()
    elif args.action == 'ports':
        print('\n'.join(ports()) or '未发现 USB 串口。请插入 USB 转串口后再运行。')
    elif args.action == 'flash':
        flash(args.probe)
    elif args.action == 'debug-load':
        # 用户按 F5 即发起编译/下载；仅由 VS Code 的 preLaunchTask 调用。
        flash(args.probe, confirm=False)
    elif args.action == 'probes':
        available = probe_list()
        print(json.dumps(available, ensure_ascii=False, indent=2) if available else '没有检测到调试器。先连接 CMSIS-DAP 的 USB。')
    elif args.action == 'console':
        console(args.port)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        sys.exit(exc.returncode)
