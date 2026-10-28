"""Strict decoders for Emm V5 Rev1.3 read-only replies. No hardware writes here.

Protocol: https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html
Unknown configuration layouts must be retained as raw data, never guessed.
"""
import re


def rx_bytes(text):
    chunks = re.findall(r'(?m)^RX: *([^\r\n]*)', text)
    return bytes.fromhex(' '.join(chunks))


def frame(text, address, opcode, length):
    if address not in range(1, 5):
        raise ValueError('Only chassis addresses 1..4')
    data = rx_bytes(text)
    candidates = [data[i:i+length] for i in range(len(data))
                  if data[i:i+2] == bytes([address, opcode]) and
                  len(data[i:i+length]) == length and data[i+length-1] == 0x6B]
    if len(candidates) != 1:
        raise ValueError('Missing, malformed or ambiguous reply')
    return candidates[0]


def config(text, address):
    p = frame(text, address, 0x42, 33)
    if p[2:4] != bytes([33, 21]) or p[20] != address:
        raise ValueError('Unsupported configuration layout/address')
    u16 = lambda i: int.from_bytes(p[i:i+2], 'big')
    return dict(motor_type=p[4], pulse_mode=p[5], serial_mode=p[6],
                enable_level=p[7], direction=p[8], microsteps=p[9] or 256,
                interpolation=p[10], screen_off=p[11], open_loop_current_ma=u16(12),
                closed_loop_max_current_ma=u16(14), closed_loop_max_voltage_mv=u16(16),
                uart_baud_code=p[18], can_baud_code=p[19], address=p[20],
                checksum_code=p[21], response_mode=p[22], stall_protection=p[23],
                stall_speed_rpm=u16(24), stall_current_ma=u16(26),
                stall_time_ms=u16(28), arrival_window_tenth_deg=u16(30))


def pid(text, address):
    p = frame(text, address, 0x21, 15)
    return {key: int.from_bytes(p[i:i+4], 'big')
            for key, i in [('kp', 2), ('ki', 6), ('kd', 10)]}
