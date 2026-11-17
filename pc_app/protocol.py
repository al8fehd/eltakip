"""Wire protocol shared with the firmware (deneyap_hand_tracker.ino).

Frame layout (multi-byte values little-endian) — mirrors the firmware:

    0xA5 0x5A | type(1) | seq(1) | len(1) | payload[len] | crc_lo(1) | crc_hi(1)

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)
covers [type, seq, len, payload].

Types:
    0x01 pose : flags(1) | status_lo(1) | status_hi(1) | 11 quaternions
                each quaternion = w, x, y, z as int16 Q15 (/32767 -> float)
                index 0 = IMU_0 (hand reference), 1..10 = IMU_1..IMU_10
    0x02 log  : ASCII text payload (firmware diagnostics)

Status bits: bit i of status_lo -> IMU_i (0..7), bit (i-8) of status_hi -> IMU_i
(8..10); set means the IMU is currently healthy.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

SYNC1 = 0xA5
SYNC2 = 0x5A
TYPE_POSE = 0x01
TYPE_LOG = 0x02

CRC_INIT = 0xFFFF
CRC_POLY = 0x1021

Q15 = 32767.0
IMU_COUNT = 11
HEADER_BYTES = 5          # sync2 + type + seq + len
CRC_BYTES = 2
MAX_PAYLOAD = 247


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE over `data`; must match the firmware exactly."""
    crc = CRC_INIT
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _decode_quat(blob: bytes) -> tuple:
    """int16 LE Q15 quadruplet -> normalized (w, x, y, z) with w >= 0."""
    w, x, y, z = struct.unpack("<4h", blob)
    q = (w / Q15, x / Q15, y / Q15, z / Q15)
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    q = tuple(v / n for v in q)
    if q[0] < 0.0:                        # sign flip: same rotation, stable interpolation
        q = tuple(-v for v in q)
    return q


@dataclass
class Frame:
    ftype: int
    seq: int
    payload: bytes

    # Decoded for type 0x01:
    flags: int = 0
    status_lo: int = 0
    status_hi: int = 0
    quats: list = field(default_factory=list)   # 11 x (w, x, y, z)

    @property
    def status(self) -> list:
        """11-element per-IMU health mask (True = healthy)."""
        bits = self.status_lo | (self.status_hi << 8)
        return [bool(bits & (1 << i)) for i in range(IMU_COUNT)]


class FrameParser:
    """Incremental stream parser: feed() arbitrary byte chunks, get frames.

    Robust against mid-stream garbage and partial frames: bytes are skipped
    one at a time until a sync pair appears; frames failing the CRC are
    discarded and resync happens after their boundary. Counters expose the
    link quality for the health HUD.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.dropped_bytes = 0     # bytes skipped while resyncing
        self.bad_crc = 0           # frames rejected on checksum
        self.frame_count = 0       # valid frames produced

    def feed(self, chunk: bytes) -> list:
        out = []
        self._buf += chunk
        while True:
            i = self._find_sync()
            if i < 0:
                # No complete sync pair (yet). Keep only a lone trailing 0xA5
                # that might open a pair with the next chunk.
                keep = 1 if (len(self._buf) and self._buf[-1] == SYNC1) else 0
                self.dropped_bytes += len(self._buf) - keep
                if keep:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                return out
            if i > 0:
                self.dropped_bytes += i
                del self._buf[:i]
            if len(self._buf) < HEADER_BYTES:
                return out
            flen = self._buf[4]
            total = HEADER_BYTES + flen + CRC_BYTES
            if len(self._buf) < total:
                return out
            frame_bytes = bytes(self._buf[:total])
            del self._buf[:total]

            ftype, seq = frame_bytes[2], frame_bytes[3]
            payload = frame_bytes[5:5 + flen]
            crc = frame_bytes[-2] | (frame_bytes[-1] << 8)
            if crc16_ccitt(frame_bytes[2:2 + 3 + flen]) != crc:
                self.bad_crc += 1
                continue
            self.frame_count += 1

            if ftype == TYPE_POSE and flen >= 3 + IMU_COUNT * 8:
                f = Frame(ftype, seq, payload)
                f.flags = payload[0]
                f.status_lo = payload[1]
                f.status_hi = payload[2]
                raw = payload[3:3 + IMU_COUNT * 8]
                f.quats = [_decode_quat(raw[k * 8:(k + 1) * 8]) for k in range(IMU_COUNT)]
                out.append(f)
            elif ftype == TYPE_LOG:
                out.append(Frame(ftype, seq, payload))

    def _find_sync(self) -> int:
        b = self._buf
        for i in range(len(b) - 1):
            if b[i] == SYNC1 and b[i + 1] == SYNC2:
                return i
        return -1
