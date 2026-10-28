"""检查实际生成的 F407 固件和向量，完全不连接硬件。"""
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import project

_, gcc, _ = project.build_environment()
binary = (ROOT/'build/chassis-f407-test.bin').read_bytes()
elf = (ROOT/'build/chassis-f407-test.elf').read_bytes()
assert elf[:4] == b'\x7fELF' and elf[4] == 1 and elf[5] == 1
assert struct.unpack_from('<H', elf, 18)[0] == 40  # ARM
assert struct.unpack_from('<I', elf, 36)[0] & 0x400  # hard-float ABI
vectors = struct.unpack_from('<' + 'I'*100, binary)
symbols = {}
for line in subprocess.check_output([str(gcc.with_name('arm-none-eabi-nm')),
                                   str(project.FIRMWARE)], text=True).splitlines():
    parts = line.split()
    if len(parts) == 3:
        symbols[parts[2]] = int(parts[0], 16)
assert vectors[0] == 0x20020000  # 128 KiB SRAM，不误算成连续 192 KiB。
for index, name in [(1, 'Reset_Handler'), (15, 'SysTick_Handler'),
                    (16+37, 'USART1_IRQHandler'), (16+52, 'UART4_IRQHandler'),
                    (16+39, 'USART3_IRQHandler')]:
    assert vectors[index] == (symbols[name] | 1), (index, name)
    assert symbols[name] != symbols['Default_Handler'], name
assert 0x20000000 <= symbols['_sdata'] <= symbols['_edata'] < 0x20020000
assert symbols['_ebss'] + 2048 <= 0x20020000
assert len(binary) <= 512*1024
# All loadable sections, including the Flash image of .data, must fit the F407VE.
phoff = struct.unpack_from('<I', elf, 28)[0]
phentsize, phnum = struct.unpack_from('<HH', elf, 42)
for i in range(phnum):
    kind, offset, vaddr, paddr, filesz, memsz, flags, align = struct.unpack_from('<8I', elf, phoff+i*phentsize)
    if kind != 1:
        continue
    assert ((0x08000000 <= vaddr and vaddr+memsz <= 0x08080000) or
            (0x20000000 <= vaddr and vaddr+memsz <= 0x20020000)), hex(vaddr)
    if filesz:
        assert 0x08000000 <= paddr and paddr+filesz <= 0x08080000, hex(paddr)
for marker in [b'CHASSIS F407 TEST v2.0 | STM32F407VET6 | IDLE',
               b'Motor: UART4 PA0/PA1',
               b'Firmware: v2.0 | Mecanum: X | Group: forward/back/left/right/cw/ccw',
               b'IMU USART3 PD8/PD9 RX_ONLY | baud=',
               b'YAW trace: moving only | interval_ms=200 | heading control=OFF']:
    assert marker in binary, marker
assert b'CHASSIS F103C8 TEST' not in binary
run = project.RUN_FILE.read_text()
for text in ['device: STMicroelectronics::STM32F407VETx', 'name: CMSIS-DAP@pyOCD',
             'protocol: swd', 'clock: 1000000', 'STM32F4xx_512.FLM']:
    assert text in run, text
print('PASS: F407 v2.0 hard-float ELF, 128 KiB SRAM/512 KiB Flash, load segments, startup/USART1/UART4/USART3 vectors, CMSIS-DAP configuration.')
