"""
Parameterized end-effector curve definition.

The curve is traced in the plane  x = X_PLANE  of the world frame
(i.e. the plane perpendicular to the x-axis of the robot base).

Edit this file to change the shape, position, orientation and speed
without touching the publisher node.

Coordinate convention (world frame):
  x  →  forward (away from base)
  y  →  lateral
  z  →  vertical (up)
"""

import numpy as np

# ── Plane & speed ──────────────────────────────────────────────────────────────

# Distance from the base along the x-axis where the curve is drawn (metres)
X_PLANE: float = 0.12

# Time to complete one full traversal of the curve (seconds)
PERIOD: float = 10.0

# ── End-effector orientation ───────────────────────────────────────────────────
# Expressed as a unit quaternion (x, y, z, w) in the world frame.
# Default: gripper pointing in the +x direction (into the plane).
#   Rz(0) * Ry(-90°) → (x=0, y=-0.7071, z=0, w=0.7071)
EE_ORIENTATION: tuple = (0.0, -0.7071, 0.0, 0.7071)


# ── Curve parameterization ─────────────────────────────────────────────────────

def curve(t: float) -> tuple[float, float]:
    """Return (y, z) coordinates for normalised parameter t ∈ [0, 1].

    Replace the body of this function to draw any shape you like.
    The coordinates are in metres in the world frame, evaluated at
    x = X_PLANE.

    Built-in examples (uncomment one):
      • Circle  (default)
      • Figure-eight / lemniscate
      • Square wave
    """

    # ── Circle ─────────────────────────────────────────────────────────────────
    radius   = 0.05    # m
    y_center = 0.0     # m
    z_center = 0.18    # m

    angle = 2.0 * np.pi * t
    y = y_center + radius * np.cos(angle)
    z = z_center + radius * np.sin(angle)
    return y, z

    # ── Figure-eight (lemniscate of Bernoulli) ─────────────────────────────────
    # scale    = 0.05
    # y_center = 0.0
    # z_center = 0.18
    # angle = 2.0 * np.pi * t
    # y = y_center + scale * np.sin(angle)
    # z = z_center + scale * np.sin(angle) * np.cos(angle)
    # return y, z

    # ── Square (rounded with np.clip) ─────────────────────────────────────────
    # side     = 0.08          # half-side length (m)
    # y_center = 0.0
    # z_center = 0.18
    # # map [0,1] to one full square traversal
    # phase = t * 4.0          # 0-1 per side
    # side_idx = int(phase) % 4
    # s = phase - int(phase)   # fractional position along current side
    # corners = [( side, -side), ( side,  side),
    #            (-side,  side), (-side, -side)]
    # p0 = np.array(corners[side_idx])
    # p1 = np.array(corners[(side_idx + 1) % 4])
    # pt = p0 + s * (p1 - p0)
    # return y_center + pt[0], z_center + pt[1]
