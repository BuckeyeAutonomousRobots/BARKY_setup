"""Version 1 transport. No ROS imports; usable in standalone tests."""
import math
import re

MAX_LINE = 255  # excludes newline


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(payload):
    raw = payload.encode('ascii')
    if any(c < 32 or c > 126 or c in b'$*' for c in raw):
        raise ValueError('invalid payload characters')
    result = b'$' + raw + f'*{crc16(raw):04X}\n'.encode('ascii')
    if len(result) - 1 > MAX_LINE:
        raise ValueError('frame too long')
    return result


def decode(line):
    if line.endswith(b'\n'):
        line = line[:-1]
    if len(line) > MAX_LINE or not line.startswith(b'$'):
        raise ValueError('bad frame')
    try:
        payload, checksum = line[1:].split(b'*')
        if not re.fullmatch(rb'[0-9A-Fa-f]{4}', checksum):
            raise ValueError('bad checksum field')
        if crc16(payload) != int(checksum, 16):
            raise ValueError('checksum mismatch')
        if any(c < 32 or c > 126 or c == ord('$') for c in payload):
            raise ValueError('invalid payload')
        return payload.decode('ascii').split(',')
    except (UnicodeError, ValueError) as exc:
        raise ValueError('invalid frame') from exc


class Framer:
    """Bounded parser; a fresh '$' resynchronizes after corruption."""
    def __init__(self):
        self.buffer = bytearray()
        self.active = False
        self.errors = 0

    def feed(self, data):
        packets = []
        for byte in data:
            if byte == ord('$'):
                self.buffer = bytearray(b'$')
                self.active = True
            elif self.active and byte == 10:
                try:
                    packets.append(decode(bytes(self.buffer)))
                except ValueError:
                    self.errors += 1
                self.buffer.clear()
                self.active = False
            elif self.active:
                self.buffer.append(byte)
                if len(self.buffer) > MAX_LINE:
                    self.errors += 1
                    self.buffer.clear()
                    self.active = False
        return packets


def uint32(value):
    if not re.fullmatch(r'[0-9]+', value) or not 0 <= int(value) <= 0xFFFFFFFF:
        raise ValueError('invalid uint32')
    return int(value)


def finite_values(values):
    result = [float(v) for v in values]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('nonfinite number')
    return result


def command(seq, values):
    values = finite_values(values)
    # Transport sanity bound ONLY. Not calibrated mechanical limits.
    if len(values) != 12 or any(abs(v) > 3.0 for v in values):
        raise ValueError('expected 12 finite test angles between -3 and +3 rad')
    uint32(str(seq))
    return encode('C,' + str(seq) + ',' + ','.join(f'{v:.6f}' for v in values))


def telemetry(fields):
    kind = fields[0]
    if kind == 'I' and len(fields) == 8:
        return kind, (uint32(fields[1]), finite_values(fields[2:]))
    if kind == 'S' and len(fields) == 9:
        if fields[1] != '1':
            raise ValueError('unsupported protocol version')
        numbers = [uint32(v) for v in fields[2:]]
        # uptime, imu_ok, link_alive, have_command, last_seq, rx_errors, tx_drops
        if any(v not in (0, 1) for v in numbers[1:4]):
            raise ValueError('invalid boolean')
        return kind, numbers
    if kind == 'A' and len(fields) == 3 and fields[2] == 'DRY_RUN':
        return kind, uint32(fields[1])
    raise ValueError('unknown telemetry schema')
