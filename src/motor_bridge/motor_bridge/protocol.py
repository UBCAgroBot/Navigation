"""
ASCII command link to the STM32 motor controller.

Must match Core/Inc/cmd_link.h in the Motorcontrols firmware.

Jetson -> STM32:  $C,<seq>,<speed_mm_s>,<steer_mrad>*<CS>\\n
STM32 -> Jetson:  $A,<seq>,<speed_mm_s>,<steer_mrad>,<flags>*<CS>\\n
                  $E,<code>*<CS>\\n

<CS> is the XOR of every byte between '$' and '*', as two uppercase hex digits.
Speed is mm/s (forward positive), steering is milliradians (left positive).
"""

from dataclasses import dataclass

ACK_FLAG_MOTORS_ENABLED = 0x01
ACK_FLAG_CLAMPED = 0x02

ERROR_CODES = {1: 'bad checksum', 2: 'bad format', 3: 'line too long'}


@dataclass
class Ack:
    seq: int
    speed_mm_s: int
    steer_mrad: int
    flags: int


@dataclass
class Error:
    code: int

    @property
    def reason(self):
        return ERROR_CODES.get(self.code, 'unknown')


def checksum(payload: str) -> int:
    cs = 0
    for b in payload.encode('ascii'):
        cs ^= b
    return cs


def encode_command(seq: int, speed_mps: float, steer_rad: float) -> bytes:
    payload = 'C,%d,%d,%d' % (
        seq & 0xFFFF, round(speed_mps * 1000.0), round(steer_rad * 1000.0))
    return ('$%s*%02X\n' % (payload, checksum(payload))).encode('ascii')


def parse_reply(line: bytes):
    """Return an Ack or Error for a valid reply line, or None if it is malformed."""
    try:
        text = line.decode('ascii').strip()
    except UnicodeDecodeError:
        return None
    if len(text) < 5 or text[0] != '$' or text[-3] != '*':
        return None

    payload = text[1:-3]
    try:
        if int(text[-2:], 16) != checksum(payload):
            return None
        fields = payload.split(',')
        if fields[0] == 'A' and len(fields) == 5:
            return Ack(*(int(f) for f in fields[1:]))
        if fields[0] == 'E' and len(fields) == 2:
            return Error(int(fields[1]))
    except ValueError:
        return None
    return None
