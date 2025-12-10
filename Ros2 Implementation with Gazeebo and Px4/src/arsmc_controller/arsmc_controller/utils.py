"""Utility functions for ARSMC controller."""

import math


def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp value between lo and hi."""
    return max(lo, min(hi, v))


def quaternion_to_yaw(q) -> float:
    """Extract yaw angle from quaternion [w, x, y, z]."""
    if len(q) < 4:
        return 0.0
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
