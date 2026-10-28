import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import driver_parameters as p


def log(data):
    # UART bytes may arrive in separate console lines.
    return '\r\n'.join('RX: '+data[i:i+7].hex(' ') for i in range(0, len(data), 7))


class ParametersTest(unittest.TestCase):
    def test_documented_layout_and_units(self):
        raw=bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 03 E8 0B B8 0F A0 05 07 01 00 01 01 00 28 09 60 0F A0 00 01 6B')
        c=p.config(log(raw),1)
        self.assertEqual(c['arrival_window_tenth_deg'],1)
        self.assertEqual(c['closed_loop_max_current_ma'],3000)
        self.assertEqual(c['stall_time_ms'],4000)
        self.assertEqual(c['microsteps'],16)
        with self.assertRaises(ValueError):p.config(log(raw),4)
        with self.assertRaises(ValueError):p.config(log(raw[:-1]),1)
        changed=bytearray(raw);changed[3]=22
        with self.assertRaises(ValueError):p.config(log(changed),1)
        with self.assertRaises(ValueError):p.config(log(raw+raw),1)

    def test_pid_unsigned_and_split(self):
        raw=bytes.fromhex('04 21 FF FF FF FF 00 00 00 64 00 00 F2 30 6B')
        self.assertEqual(p.pid(log(raw),4),dict(kp=4294967295,ki=100,kd=62000))
        changed=bytearray(raw);changed[-1]=0
        with self.assertRaises(ValueError):p.pid(log(changed),4)
        with self.assertRaises(ValueError):p.pid(log(raw),0)


if __name__=='__main__':unittest.main()
