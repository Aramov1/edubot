# Task 4 — Pick-and-Place with Position-Based Impedance Control

## Overview

This node performs a bidirectional pick-and-place operation (A→B, then B→A) on the SO-ARM100
using **Cartesian-space proportional velocity control with directional stiffness gains** —
a position-based impedance analogue that requires no force/torque sensor.

The key design choice is `k_z << k_xy`: the Z-axis is 3–4× softer than XY, so the robot
descends gently onto a surface rather than impacting it.  This provides implicit force
regulation without torque control.

---

## Run Command

```bash
ros2 run assignment pick_place_impedance
```

The node auto-launches `lerobot_sim` in velocity mode if it is not already running.

---

## Configuration

Edit `config/pick_place_config.json` before running:

| Key | Description |
|-----|-------------|
| `locations.A / B` | Pick and place positions in metres (adjust to your physical grid) |
| `home.joint_angles_rad` | Home configuration (all zeros = upright) |
| `motion.approach_z` | Z height for approach/lift waypoints (m) |
| `motion.threshold_*_m` | Distance thresholds for state transitions (m) |
| `motion.dwell_grasp_s / dwell_release_s` | Hold time after gripper close/open (s) |
| `motion.descent_timeout_s` | Max time allowed for descent before forcing transition (s) |
| `motion.max_cycles` | Number of pick-and-place cycles (2 = A→B then B→A) |
| `impedance.k_xy` | XY stiffness gain (higher = faster lateral moves) |
| `impedance.k_z` | Z stiffness gain (lower = gentler descent) |
| `impedance.k_rot` | Rotational stiffness gain |
| `impedance.d_xy / d_z / d_rot` | Damping gains (reduce oscillation) |
| `impedance.max_cart_vel_m_s` | Cartesian velocity safety clamp (m/s) |
| `impedance.max_joint_vel_rad_s` | Joint velocity safety clamp (rad/s) |
| `gripper.open_angle_rad` | Fully open gripper angle (rad) |
| `gripper.close_angle_rad` | Grasp angle — tune for your object size |
| `gripper.vel_rad_s` | Gripper servo speed |
| `gripper.dead_band_rad` | Angular dead-band for gripper settled check |

---

## Control Architecture

```
e_pos  = x_des - x_cur           # (6,) Cartesian pose error [m, rad]
e_vel  = -dx_cur                  # (6,) damping (target velocity = 0)
v_cart = K_m @ e_pos + D_m @ e_vel
v_cart = clip(v_cart, ±max_cart_vel)
dq     = J_inv @ v_cart           # (5,) joint velocities via Jacobian pseudo-inverse
dq     = clip(dq, ±max_joint_vel)
```

- **K_m** = `diag([k_xy, k_xy, k_z, k_rot, k_rot, k_rot])` — directional stiffness
- **D_m** = `diag([d_xy, d_xy, d_z, d_rot, d_rot, d_rot])` — velocity damping
- During translation states (APPROACH, DESCEND, LIFT, MOVE) the orientation target equals
  the current EE orientation, so only XYZ error is controlled.
- The Jacobian pseudo-inverse uses DLS (Damped Least Squares) for singularity robustness.

### Phase 2 Hybrid Upgrade

`ImpedanceController.compute()` accepts a `contact_mask` argument (Phase 1: all zeros).
To add force control on the Z-axis during descent:
1. Subscribe to a `/force_torque` topic.
2. Set `contact_mask[2] = True` during DESCEND states.
3. Replace `v_cart[i]` with `K_f[i,i] * (f_des[i] - f_cur[i])` for masked axes.

---

## State Machine

```
HOME → APPROACH_OBJECT → DESCEND_TO_OBJECT → GRASP → LIFT
     → MOVE_TO_TARGET → DESCEND_TO_PLACE → RELEASE → LIFT_AFTER_PLACE
     → RETURN_HOME → SWAP_LOCATIONS → [HOME again if cycle < max_cycles]
                                     → DONE
```

Cycle 1: pick A → place B | Cycle 2: pick B → place A

---

## Tuning Guide

| Symptom | Remedy |
|---------|--------|
| Oscillation during lateral moves | Reduce `k_xy` or increase `d_xy` |
| Arm hits surface too hard | Reduce `k_z` (try 0.3–0.5) |
| Slow approach | Increase `k_xy` / `max_cart_vel_m_s` |
| Gripper slips | Decrease `close_angle_rad` |
| Gripper crushes object | Increase `close_angle_rad` |
| Velocity explosion near singularity | Already handled by DLS; check log for warnings |

---

## Assumptions & Conventions

- All positions are in the robot's world frame (metres).
- Angles are in radians throughout.
- `snake_case` naming for all Python identifiers.
- `/joint_states` `effort` field is always empty on this robot; force regulation is achieved
  by conservative `close_angle_rad` and low `k_z`.
- The gripper is the 6th element of `JointTrajectory.velocities` (robot.cpp:115-127).
