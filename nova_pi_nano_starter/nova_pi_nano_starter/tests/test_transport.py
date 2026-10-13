"""Run from project root: python3 -m unittest discover -s tests -v"""
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/nova_hardware'))
from nova_hardware.protocol import Framer, command, crc16, decode, encode, telemetry


class TransportTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)

    def test_partial_and_combined_frames(self):
        frame = encode('H,42')
        parser = Framer()
        self.assertEqual(parser.feed(frame[:3]), [])
        self.assertEqual(parser.feed(frame[3:] + frame), [['H', '42'], ['H', '42']])

    def test_corruption_and_resynchronization(self):
        frame = encode('H,42').replace(b'42', b'43')
        parser = Framer()
        self.assertEqual(parser.feed(b'noise' + frame + encode('H,7')), [['H', '7']])
        self.assertEqual(parser.errors, 1)

    def test_oversize_and_embedded_start(self):
        parser = Framer()
        self.assertEqual(parser.feed(b'$' + b'x' * 300 + b'\n$incomplete' + encode('H,1')), [['H', '1']])
        self.assertEqual(parser.errors, 1)

    def test_command_validation(self):
        self.assertEqual(len(decode(command(1, [0.0] * 12))), 14)
        for values in ([0.0] * 11, [math.nan] * 12, [math.inf] * 12, [3.1] * 12):
            with self.assertRaises(ValueError):
                command(1, values)

    def test_telemetry_validation(self):
        for payload in ('I,0,nan,0,0,0,0,0', 'S,2,0,1,1,0,0,0,0',
                        'S,1,0,2,1,0,0,0,0', 'A,-1,DRY_RUN', 'A,0,ARMED'):
            with self.assertRaises(ValueError):
                telemetry(decode(encode(payload)))


@unittest.skipUnless(shutil.which('g++'), 'g++ required for firmware host harness')
class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.executable = str(Path(cls.temp.name) / 'firmware_test')
        subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(ROOT / 'tests/stubs'), '-I' + str(ROOT / 'firmware/include'), str(ROOT / 'tests/firmware_harness.cpp'),
                        '-o', cls.executable], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_firmware(self, raw):
        output = subprocess.run([self.executable], input=raw, capture_output=True, check=True).stdout
        parser = Framer()
        packets = [telemetry(fields) for fields in parser.feed(output)]
        self.assertEqual(parser.errors, 0)
        return packets

    def test_cross_language_ack_and_si_units(self):
        packets = self.run_firmware(encode('H,1') + command(42, [0.25] * 12))
        self.assertIn(('A', 42), packets)
        imu = next(data for kind, data in packets if kind == 'I')[1]
        self.assertAlmostEqual(imu[2], 9.80665, places=5)
        self.assertAlmostEqual(imu[3], math.pi / 2, places=5)

    def test_watchdog_expires_without_heartbeat(self):
        statuses = [data for kind, data in self.run_firmware(encode('H,1')) if kind == 'S']
        self.assertEqual(statuses[0][2], 1)
        self.assertEqual(statuses[-1][2], 0)

    def test_bad_commands_do_not_acknowledge_or_refresh_link(self):
        bad_payloads = ['C,1,' + ','.join(['nan'] * 12),
                        'C,2,' + ','.join(['3.1'] * 12),
                        'C,3,' + ','.join(['0'] * 11),
                        'C,4,' + ','.join(['0'] * 13),
                        'C,5,,0,0,0,0,0,0,0,0,0,0,0', 'H,-1']
        for payload in bad_payloads:
            with self.subTest(payload=payload):
                packets = self.run_firmware(encode(payload))
                self.assertFalse(any(kind == 'A' for kind, _ in packets))
                status = [data for kind, data in packets if kind == 'S'][-1]
                self.assertEqual(status[2], 0)
                self.assertGreater(status[5], 0)

    def test_noise_and_crc_error_then_valid_command(self):
        broken = command(10, [0.0] * 12).replace(b'C,10', b'C,11')
        packets = self.run_firmware(b'$' + b'x' * 300 + b'\n' + broken + command(12, [0.0] * 12))
        self.assertIn(('A', 12), packets)
        self.assertNotIn(('A', 11), packets)


if __name__ == '__main__':
    unittest.main()
