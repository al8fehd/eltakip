"""Pose math: relative quaternions, finger joints, neutral-pose subtraction.

Input : 11 world-frame quaternions from the firmware (protocol.Frame).
Output: JSON-ready pose dicts built on the invariant that rotating the whole
        hand never bends a finger:

            q_rel = q_hand^-1 (x) q_seg                (segment in hand frame)
            prox  = q_rel(lower)
            dist  = q_rel(upper) (x) q_rel(lower)^-1   (upper in lower frame)

A neutral "zero pose" is captured (automatically at startup, or on UI
command) and subtracted per joint, so mounting offsets of the arbitrary glove
segments cancel out and the zero pose renders the anatomical flat open hand.

Phase 2/3 note: pose["segments"] always carries the raw, neutral-independent
relative quaternions; a future gesture classifier subscribes to the pose
dict and gets everything it needs without any Phase 1 refactor.
"""

from __future__ import annotations

import math
import time

import protocol

# Finger -> (lower IMU index, upper IMU index); see firmware header comment.
FINGERS = {
    "thumb": (1, 2),
    "index": (3, 4),
    "middle": (5, 6),
    "ring": (7, 8),
    "pinky": (9, 10),
}

SEGMENT_KEYS = [f"{f}_{s}" for f in FINGERS for s in ("lower", "upper")]
JOINT_KEYS = [f"{f}_{s}" for f in FINGERS for s in ("prox", "dist")]
SEG_IMU = {name: idx for f, (lo, hi) in FINGERS.items()
           for name, idx in ((f + "_lower", lo), (f + "_upper", hi))}

# Optional static alignment for IMU_0 on the hand dorsum (identity = none).
# If the sensor axes do not match the hand frame (X toward fingertips,
# Y through the dorsum, Z toward the pinky), replace this with the correcting
# rotation. Per-segment mounting discrepancies are absorbed by the neutral
# pose capture instead, so this is only about the *global* hand orientation.
HAND_FIX = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Quaternion algebra (w, x, y, z) — stdlib only, no numpy.
# ---------------------------------------------------------------------------

def qmul(a, b):
    """Hamilton product: rotate by b first, then by a."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def qinv(q):
    w, x, y, z = q
    n2 = w * w + x * x + y * y + z * z
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n2
    return (w * inv, -x * inv, -y * inv, -z * inv)


def qnorm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    q = tuple(v / n for v in q)
    if q[0] < 0.0:
        q = tuple(-v for v in q)
    return q


def qaxis_angle(ux, uy, uz, angle):
    """Unit axis (ux, uy, uz), angle in radians."""
    s = math.sin(0.5 * angle)
    return qnorm((math.cos(0.5 * angle), ux * s, uy * s, uz * s))


def qcompose(qs):
    """Concatenate rotations; the first quaternion is applied last (parent frame)."""
    acc = (1.0, 0.0, 0.0, 0.0)
    for q in qs:
        acc = qmul(acc, q)
    return acc


def q_to_list(q):
    return [round(v, 6) for v in q]


# ---------------------------------------------------------------------------
# Pose tracker
# ---------------------------------------------------------------------------

class PoseTracker:
    """Builds pose dicts from firmware frames; owns neutral-pose capture."""

    def __init__(self, hand_fix=HAND_FIX, auto_zero=True):
        self.hand_fix = hand_fix
        self.auto_zero = auto_zero
        self.neutral = {}
        self.zeroed = False
        self.zeroed_at = None
        self.zero_source = None
        self._last_input = None          # (hand_q, seg_rel) of the newest frame

    # -- neutral pose -------------------------------------------------------

    def capture_neutral(self, hand_q, seg_rel, source="user"):
        """Pin the current relative posture as the neutral (zero) pose."""
        self.neutral = dict(self._joints_of(seg_rel))
        self.zeroed = True
        self.zeroed_at = time.time()
        self.zero_source = source

    def capture_neutral_from_last(self, source="user"):
        """Re-zero using the newest frame (used by the UI command)."""
        if self._last_input is None:
            return False
        self.capture_neutral(*self._last_input, source=source)
        return True

    # -- pose building ------------------------------------------------------

    def on_frame(self, frame) -> dict | None:
        """Decode one pose frame into a JSON-ready pose dict."""
        if frame.ftype != protocol.TYPE_POSE or len(frame.quats) < protocol.IMU_COUNT:
            return None

        hand = qmul(frame.quats[0], self.hand_fix)      # fixed hand orientation, world frame
        seg_rel = {
            name: qnorm(qmul(qinv(hand), frame.quats[idx]))
            for name, idx in SEG_IMU.items()
        }
        joints = self._joints_of(seg_rel)
        self._last_input = (hand, seg_rel)

        if not self.zeroed and self.auto_zero:
            self.capture_neutral(hand, seg_rel, source="auto")

        out_joints = {}
        for k in JOINT_KEYS:
            j = joints[k]
            if self.zeroed and k in self.neutral:
                j = qnorm(qmul(qinv(self.neutral[k]), j))   # neutral^-1 (x) current
            out_joints[k] = {"q": q_to_list(j)}

        mask = frame.status
        faulty = [f"IMU_{i}" for i, ok in enumerate(mask) if not ok]

        return {
            "type": "pose",
            "t": time.time(),
            "seq": frame.seq,
            "zeroed": self.zeroed,
            "zeroed_at": self.zeroed_at,
            "hand": {"q": q_to_list(hand)},
            "segments": {k: {"q": q_to_list(v)} for k, v in seg_rel.items()},
            "joints": out_joints,
            "health": {"n": sum(mask), "total": protocol.IMU_COUNT, "faulty": faulty},
        }

    @staticmethod
    def _joints_of(seg_rel):
        """prox = lower-segment rel; dist = upper rel in the lower frame."""
        joints = {}
        for f, (lo, hi) in FINGERS.items():
            joints[f"{f}_prox"] = qnorm(seg_rel[f"{f}_lower"])
            joints[f"{f}_dist"] = qnorm(qmul(qconj(seg_rel[f"{f}_lower"]), seg_rel[f"{f}_upper"]))
        return joints
