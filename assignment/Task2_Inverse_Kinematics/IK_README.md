# Python Controllers — IK & End-Effector Nodes

Reference guide for the three end-effector nodes in `python_controllers` and the kinematics module they depend on.

**Package location:** `ros_ws/src/python_controllers/`

---

## Table of Contents

1. [Prerequisites & Build](#1-prerequisites--build)
2. [Kinematics Foundation — `robot_kinematics.py`](#2-kinematics-foundation--robot_kinematicspy)
3. [Node: `read_ee_pose`](#3-node-read_ee_pose)
4. [Node: `set_joint_position`](#4-node-set_joint_position)
5. [Node: `trajectory_follower`](#5-node-trajectory_follower)
6. [Trajectory File — `trajectories.json`](#6-trajectory-file--trajectoriesjson)
7. [Workflow Combinations](#7-workflow-combinations)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Prerequisites & Build

**Required ROS2 packages** (installed with the workspace):
```
rclpy  sensor_msgs  trajectory_msgs  geometry_msgs  tf2_ros
```

**Required Python packages:**
```
numpy  scipy  sympy
```

**Build:**
```bash
cd ~/TU_Delft/edubot/ros_ws
colcon build --packages-select python_controllers
source install/setup.bash
```

> Always `source install/setup.bash` after a build or in every new terminal before running any node.

---

## 2. Kinematics Foundation — `robot_kinematics.py`

**File:** `assignment/robot_kinematics.py`
**Class:** `RobotKinematics`

This module is the mathematical backbone of the two task-space nodes (`trajectory_follower`). It provides symbolic forward kinematics (FK) and numerical inverse kinematics (IK) for the SO-ARM100. It is a standalone Python module — no ROS2 required.

### 2.1 How it Works Internally

At construction, the class builds a **symbolic 4×4 homogeneous transformation matrix** using SymPy, then compiles it into a fast numerical function with `lambdify`. This compilation step happens once and takes a few seconds.

The full kinematic chain (world → gripper center) is:

```
T_total = T_world_base * T_base_shoulder(q1) * T_shoulder_upperarm(q2)
        * T_upperarm_lowerarm(q3) * T_lowerarm_wrist(q4)
        * T_wrist_gripper(q5) * T_gripper_center
```

Each joint transform follows the URDF convention:  `T = Trans(xyz) * RotFixed(rpy) * Rz(q)`

| Joint | Fixed Rotation | Translation (m) | Variable |
|-------|---------------|-----------------|----------|
| World → Base | Rz(π) | (0, 0, 0) | — |
| Base → Shoulder | — | (0, −0.0452, 0.0165) | Rz(q1) |
| Shoulder → Upper Arm | Ry(−π/2) | (0, −0.0306, 0.1025) | Rz(q2) |
| Upper Arm → Lower Arm | — | (0.11257, −0.028, 0) | Rz(q3) |
| Lower Arm → Wrist | Ry(+π/2) | (0.0052, −0.1349, 0) | Rz(q4) |
| Wrist → Gripper | Ry(−π/2) | (−0.0601, 0, 0) | Rz(q5) |
| Gripper → Gripper Center | — | (0, 0, 0.075) | — |

The FK output is a 5-element pose vector: **`[x, y, z, pitch, roll]`**
- `x, y, z` — position in metres, world frame
- `pitch` — `arcsin(−R[2,0])` — rotation around Y axis (rad)
- `roll` — `atan2(R[2,1], R[2,2])` — rotation around X axis (rad)

### 2.2 Joint Limits

| Joint | Variable | Min (rad) | Max (rad) |
|-------|----------|-----------|-----------|
| Shoulder_Rotation | q1 | −2.000 | +2.000 |
| Shoulder_Pitch | q2 | −π/2 ≈ −1.571 | +π/2 ≈ +1.571 |
| Elbow | q3 | −π/2 ≈ −1.571 | +π/2 ≈ +1.571 |
| Wrist_Pitch | q4 | −π/2 ≈ −1.571 | +π/2 ≈ +1.571 |
| Wrist_Roll | q5 | −π/2 ≈ −1.571 | +π/2 ≈ +1.571 |

### 2.3 API Reference

#### `forward_kinematics(q1, q2, q3, q4, q5)`
Computes the end-effector pose from joint angles.

| | Type | Description |
|---|---|---|
| **Input** | 5 floats (radians) | Joint angles within bounds |
| **Output** | `np.array` shape `(5,)` | `[x, y, z, pitch, roll]` — metres and radians |
| **Raises** | `ValueError` | If any joint is outside its limits |

```python
from robot_kinematics import RobotKinematics
kin = RobotKinematics()

pose = kin.forward_kinematics(0.0, -0.5, 0.3, 0.2, 0.0)
# → array([x, y, z, pitch, roll])
print(f'EE at x={pose[0]:.3f}  y={pose[1]:.3f}  z={pose[2]:.3f}')
```

#### `inverse_kinematics(target_pose, n_restarts=15)`
Finds joint angles that place the EE at the desired pose.

| | Type | Description |
|---|---|---|
| **Input** | `list[float]` length 5 | `[x, y, z, pitch, roll]` — metres and radians |
| **Output** | `list[float]` length 5 | Joint angles `[q1..q5]` in radians |
| **Failure** | `[nan, nan, nan, nan, nan]` | Target is outside the reachable workspace |

The solver uses **SLSQP** constrained optimization. It runs `n_restarts + 1` times from different random starting configurations to avoid local minima. Weights: position error × 10, orientation error × 1. Convergence tolerance: `1e-6`. It returns early if error < `1e-7`.

```python
q = kin.inverse_kinematics([0.12, 0.0, 0.18, 0.0, 0.0])
if not any(np.isnan(q)):
    print(f'Solution: {np.round(q, 3)} rad')
else:
    print('Target unreachable')
```

> **Performance:** The first call after creating `RobotKinematics()` is slow (~2–5 s) due to SymPy's JIT compilation. All subsequent calls are fast. Pre-compute IK at startup, not inside real-time loops.

---

## 3. Node: `read_ee_pose`

**File:** `python_controllers/read_ee_pose.py`
**Node name:** `read_ee_pose`
**Purpose:** Continuously reads and prints the end-effector Cartesian position and orientation to the terminal.

### 3.1 How it Works

The node does **not** compute kinematics itself. It reads from the TF tree that `robot_state_publisher` maintains. Every 100 ms it queries the transform from the `world` frame to the `gripper_center` frame, converts the quaternion to roll/pitch/yaw in degrees, and logs the result.

```
/joint_states ──> robot_state_publisher ──> /tf
                  (computes FK, URDF-based)     │
                                                └──> read_ee_pose
                                                     (queries TF, prints pose)
```

### 3.2 Topics & Dependencies

| Direction | Topic / Frame | Type | Description |
|-----------|--------------|------|-------------|
| Reads | TF: `world` → `gripper_center` | `tf2_ros` | Full 6-DOF EE pose |
| Requires | `robot_state_publisher` running | — | Publishes TF from `/joint_states` |

No parameters. No ROS topics published.

### 3.3 Usage

```bash
# Terminal 1 — any robot launch (provides robot_state_publisher)
ros2 launch lerobot sim_position.launch.py

# Terminal 2
ros2 run python_controllers read_ee_pose
```

**Output (10 Hz):**
```
[read_ee_pose]:
EE Position  (m)  : x=+0.1200  y=+0.0500  z=+0.1800 | Orientation (deg) : roll=  +0.00  pitch= +90.00  yaw=  +0.00
```

### 3.4 Useful Companion Commands

```bash
# One-shot from terminal (no node needed)
ros2 run tf2_ros tf2_echo --once world gripper_center

# Stream continuously
ros2 run tf2_ros tf2_echo world gripper_center

# Visualise all TF frames
ros2 run tf2_tools view_frames
```

---

## 4. Node: `set_joint_position`

**File:** `python_controllers/set_joint_position.py`
**Node name:** `set_joint_position`
**Purpose:** Interactive terminal controller. You type joint angles in degrees; the robot moves to them. Useful for manual testing, calibration, and exploring the workspace.

### 4.1 How it Works

The node uses two concurrent threads:

- **ROS spin thread** — runs `rclpy.spin()`, publishes the current target to `/joint_cmds` at **10 Hz** continuously. This keeps the robot holding its last commanded position even when no new input is given.
- **Input thread** — blocks on `input()` waiting for user commands. When a valid command is received, it updates the shared `_target` which the publish timer reads.

```
[User types in terminal]
        │
  input thread (background)
        │ validates + converts deg→rad
        │ updates self._target
        │
  timer thread (10 Hz)
        │ reads self._target
        └──> publishes /joint_cmds (JointTrajectory)
                    │
              lerobot_sim or lerobot_hw
                    │
              robot moves to target
```

### 4.2 Topics

| Direction | Topic | Type | Rate |
|-----------|-------|------|------|
| Publishes | `/joint_cmds` | `trajectory_msgs/JointTrajectory` | 10 Hz |

### 4.3 Joint Limits (enforced before sending)

| Index | Joint | Min (deg) | Max (deg) |
|-------|-------|-----------|-----------|
| 0 | Shoulder_Rotation | −115.0 | +115.0 |
| 1 | Shoulder_Pitch | −90.0 | +90.0 |
| 2 | Elbow | −90.0 | +90.0 |
| 3 | Wrist_Pitch | −90.0 | +90.0 |
| 4 | Wrist_Roll | −180.0 | +180.0 |
| 5 | Gripper | 0.0 | +90.0 (0=closed, 90=open) |

### 4.4 Usage

```bash
# Terminal 1 — start robot (sim or hardware)
ros2 launch lerobot sim_position.launch.py

# Terminal 2
ros2 run python_controllers set_joint_position
```

**Terminal interface:**
```
==========================================================
  LeRobot — Interactive Joint Position Controller
==========================================================
  Joint                    Min     Max  Unit
  ----------------------  ------  ------  ----
  Shoulder_Rotation       -115.0   115.0  deg
  Shoulder_Pitch           -90.0    90.0  deg
  Elbow                    -90.0    90.0  deg
  Wrist_Pitch              -90.0    90.0  deg
  Wrist_Roll              -180.0   180.0  deg
  Gripper                    0.0    90.0  deg
==========================================================
  Enter 6 space-separated values in degrees.
  Press Enter with no input to resend current target.
  Type  "home"  to go to the home position.
  Type  "quit"  to exit.
==========================================================

  > 0 -60 45 30 0 45
  Command sent.
  Current target: [   0.00°    -60.00°    45.00°    30.00°     0.00°    45.00° ]

  > home
  Moved to home position.

  > quit
  Shutting down.
```

### 4.5 Input Commands

| Command | Description |
|---------|-------------|
| `q0 q1 q2 q3 q4 q5` | Six space-separated values in **degrees** |
| `home` | Sends all joints to 0° (the zero configuration) |
| `<Enter>` | Resends the current target (no movement) |
| `exit` | Shuts down the node |

> **Note:** Values outside the joint limits are rejected with an error message before sending. The robot never receives an out-of-bounds command.

---

## 5. Node: `trajectory_follower`

**File:** `python_controllers/trajectory_follower.py`
**Node name:** `trajectory_follower`
**Purpose:** Autonomous task-space trajectory execution. Loads a set of Cartesian waypoints from a JSON file, solves IK for each at startup, then drives the robot along the path in position mode. Advances to the next waypoint only when the EE is close enough to the current one.

### 5.1 How it Works

```
trajectories.json
      │  (loaded at startup)
      ▼
  _load_trajectory()
      │  for each waypoint:
      │    IK([x,y,z,pitch,roll]) → [q1..q5]
      │    verify with FK, log error
      ▼
  self._joint_targets[]        self._cartesian_targets[]
  (pre-computed joint angles)  (target Cartesian positions)
      │                                │
      ▼                                │
  timer @ publish_rate Hz              │
      │                                │
      ├──> publish /joint_cmds ─────── │ ──> robot moves toward target_q
      │                                │
      └──> subscribe /joint_states     │
              │                        │
              ▼                        │
           FK(current_q) ─────────────>│
              │       compare          │
              └── error < threshold? ──┘
                        │  YES
                        ▼
                  advance waypoint index
                  (loop or stop)
```

### 5.2 ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `trajectory_file` | `<pkg_share>/config/trajectories.json` | Absolute path to the trajectory JSON file |
| `kinematics_path` | `/home/andre/TU_Delft/edubot/assignment` | Directory containing `robot_kinematics.py` |
| `publish_rate` | `10.0` | Publishing frequency in Hz |
| `gripper` | `0.0` | Fixed gripper value appended to all joint commands (0=closed) |

### 5.3 Topics

| Direction | Topic | Type | Rate | Description |
|-----------|-------|------|------|-------------|
| Publishes | `/joint_cmds` | `JointTrajectory` | `publish_rate` Hz | Current waypoint joint command |
| Subscribes | `/joint_states` | `JointState` | — | Reads current q to compute EE position via FK |

### 5.4 Usage

**Step 1 — Select trajectory** (edit one line in `trajectories.json`):
```json
"active_trajectory": "circle_yz"
```

**Step 2 — Launch the simulator:**
```bash
ros2 launch lerobot sim_position.launch.py
```

**Step 3 — Run the node:**
```bash
ros2 run python_controllers trajectory_follower
```

**Override parameters at launch:**
```bash
# Use a custom trajectory file
ros2 run python_controllers trajectory_follower --ros-args \
  -p trajectory_file:=/path/to/my_trajectories.json

# Open gripper during trajectory
ros2 run python_controllers trajectory_follower --ros-args \
  -p gripper:=1.5708

# Slower publishing rate
ros2 run python_controllers trajectory_follower --ros-args \
  -p publish_rate:=5.0
```

### 5.5 Startup Output

The node logs its progress during initialisation:
```
[trajectory_follower]: Loading trajectory file: .../trajectories.json
[trajectory_follower]: Active trajectory: "circle_yz" — Circle in the Y-Z plane...
[trajectory_follower]: Pre-computing IK for 12 waypoints (pitch=0.0°, roll=0.0°)...
[trajectory_follower]:   WP 00 [0.120,  0.050, 0.180] → q=[ 0.  -0.5  0.3  0.2  0. ]  FK err=1.2 mm
[trajectory_follower]:   WP 01 [0.120,  0.043, 0.205] → q=[ 0.  -0.4  0.2  0.3  0. ]  FK err=0.8 mm
   ...
[trajectory_follower]: Pre-computation done: 12/12 waypoints valid.
[trajectory_follower]: Trajectory follower running — 12 waypoints, loop=True, threshold=0.015 m
```

**Runtime output (one line per waypoint reached):**
```
[trajectory_follower]: Reached WP 00 [0.120,  0.050, 0.180] (err=8.3 mm)
[trajectory_follower]: Reached WP 01 [0.120,  0.043, 0.205] (err=11.2 mm)
   ...
[trajectory_follower]: Loop: restarting trajectory from WP 00.
```

### 5.6 Waypoint Advancement Logic

A waypoint is considered **reached** when:
```
‖ FK(current_q) − target_position ‖ < position_threshold
```
where `current_q` comes from the latest `/joint_states` message and `position_threshold` is set per-trajectory in the JSON (default: 0.015 m = 1.5 cm).

If `loop: true` — the sequence restarts from waypoint 0 after the last one.
If `loop: false` — the node holds the final position and stops advancing.

---

## 6. Trajectory File — `trajectories.json`

**File:** `python_controllers/config/trajectories.json`
**Installed to:** `<ros2_share>/python_controllers/config/trajectories.json`

### 6.1 Full Schema

```json
{
  "active_trajectory": "<id>",         ← REQUIRED: ID of the trajectory to execute

  "trajectories": {
    "<id>": {                           ← Unique string identifier
      "description": "...",            ← Human-readable label
      "loop": true,                    ← true: restart after last WP | false: hold last WP
      "position_threshold": 0.015,     ← Metres: when EE is this close, advance to next WP
      "orientation": {
        "pitch": 0.0,                  ← Radians: desired EE pitch (passed to IK as target)
        "roll":  0.0                   ← Radians: desired EE roll  (passed to IK as target)
      },
      "waypoints": [
        { "x": 0.12, "y":  0.05, "z": 0.18 },
        { "x": 0.12, "y":  0.00, "z": 0.23 },
        ...
      ]
    }
  }
}
```

### 6.2 Pre-defined Trajectories

| ID | Shape | x (m) | y range (m) | z range (m) | Waypoints |
|----|-------|--------|-------------|-------------|-----------|
| `circle_yz` | Circle, r=0.05 m | 0.12 | ±0.05 | 0.13–0.23 | 12 |
| `square_yz` | Square, half-side=0.04 m | 0.12 | ±0.04 | 0.14–0.22 | 16 |
| `line_z` | Vertical oscillation | 0.12 | 0 | 0.13–0.23 | 8 |

All trajectories are in the **Y-Z frontal plane at x=0.12 m** — the safe working distance validated by the assignment (`curve_definition.py`).

### 6.3 Switching Trajectories

Edit the single top-level field:
```json
"active_trajectory": "square_yz"
```
Then restart the node. No code changes needed.

### 6.4 Adding a Custom Trajectory

Add a new entry inside `"trajectories": { ... }`:
```json
"my_shape": {
  "description": "My custom 5-point diagonal",
  "loop": false,
  "position_threshold": 0.010,
  "orientation": { "pitch": 0.0, "roll": 0.0 },
  "waypoints": [
    { "x": 0.12, "y":  0.04, "z": 0.14 },
    { "x": 0.12, "y":  0.02, "z": 0.16 },
    { "x": 0.12, "y":  0.00, "z": 0.18 },
    { "x": 0.12, "y": -0.02, "z": 0.20 },
    { "x": 0.12, "y": -0.04, "z": 0.22 }
  ]
}
```

Then set `"active_trajectory": "my_shape"` and restart the node.

**Workspace guidelines for waypoints:**

| Coordinate | Safe range | Notes |
|-----------|------------|-------|
| `x` | 0.08 – 0.20 m | x=0.12 is fully validated; larger x reduces reachability |
| `y` | −0.10 – +0.10 m | Symmetric around robot centre |
| `z` | 0.10 – 0.25 m | Below 0.10 may cause elbow collision with base |

> **Tip:** If the node warns `UNREACHABLE` for a waypoint, the position is outside the robot's reachable workspace. Reduce the range or move closer to x=0.12.

---

## 7. Workflow Combinations

### Manual Exploration + Pose Reading

Use `set_joint_position` to pose the robot and `read_ee_pose` to observe the resulting Cartesian position simultaneously:

```bash
# T1 — simulator
ros2 launch lerobot sim_position.launch.py

# T2 — interactive joint controller
ros2 run python_controllers set_joint_position

# T3 — live EE position feed
ros2 run python_controllers read_ee_pose
```
Move joints in T2, watch coordinates update in T3.

---

### Autonomous Trajectory + Live Monitoring

Run a trajectory automatically while watching the EE position:

```bash
# T1 — simulator
ros2 launch lerobot sim_position.launch.py

# T2 — trajectory follower
ros2 run python_controllers trajectory_follower

# T3 — live EE position (optional, for verification)
ros2 run python_controllers read_ee_pose
```

---

### Hardware Execution

Replace the simulator launch with the hardware launch. No other changes needed:

```bash
# T1 — hardware node (position mode)
ros2 launch lerobot hw_position.launch.py

# T2 — start robot_state_publisher + RViz separately (not included in hw launch)
ros2 launch lerobot rviz.launch.py

# T3 — any of the three nodes
ros2 run python_controllers trajectory_follower
```

> **Hardware safety:** `set_joint_position` and `trajectory_follower` both operate at `max_speed=0.2 rad/s` (enforced by the hardware node). Large joint jumps in `set_joint_position` will cause the arm to sweep at maximum speed — always start from a position close to the target.

---

## 8. Troubleshooting

### `TF not available yet` (read_ee_pose)
`robot_state_publisher` is not running. Start it via any of:
```bash
ros2 launch lerobot sim_position.launch.py   # includes it
ros2 launch lerobot rviz.launch.py           # includes it
```

### `Cannot import RobotKinematics` (trajectory_follower)
The `kinematics_path` parameter does not point to the directory containing `robot_kinematics.py`. Fix:
```bash
ros2 run python_controllers trajectory_follower --ros-args \
  -p kinematics_path:=/correct/path/to/assignment
```

### `active_trajectory "xyz" not found`
The value of `active_trajectory` in the JSON does not match any key in `trajectories`. Check spelling and verify the JSON is valid with:
```bash
python3 -c "import json; json.load(open('config/trajectories.json'))"
```

### Waypoints marked `UNREACHABLE`
The position `[x, y, z]` is outside the reachable workspace. Try:
- Moving the point closer to `x=0.12` (the validated safe plane)
- Reducing the radius or extent of the trajectory
- Running `assignment/Task1_Mobility_and_Workspace/reachable_workspace.py` to visualise the workspace

### Robot does not advance past a waypoint
The `position_threshold` is too tight for the simulation accuracy. Increase it in the JSON:
```json
"position_threshold": 0.025
```

### IK is very slow at startup
This is expected for the first call — SymPy compiles the symbolic FK model into a numerical function. Subsequent calls are fast. Pre-computation happens once at node startup; it does not affect real-time performance.

---

## 9. IK Solution Analysis — Test Cases

Results from running `inv_kinematics.py` against five representative EE pose targets.
The solver uses SLSQP with 50 random restarts, `error_threshold=1e-3`, `dedup_tol=0.05 rad`.

### 9.1 Results table

| Target `[x, y, z, pitch, roll]` | Solutions found | Verdict |
|---|---|---|
| `[0.20, 0.20, 0.20, 1.571, 0.000]` | 2 | ✅ Reachable — two distinct configurations |
| `[0.20, 0.10, 0.40, 0.000, 1.571]` | 0 | ❌ Unreachable — wrist orientation infeasible |
| `[0.00, 0.00, 0.45, 0.785, 0.785]` | 0 | ❌ Unreachable — waist singularity + orientation |
| `[0.00, 0.00, 0.07, 3.141, 0.000]` | 0 | ❌ Unreachable — below shoulder + pitch = π |
| `[0.00, 0.045, 0.45, 0.785, 3.141]` | 0 | ❌ Unreachable — roll = π exceeds wrist limits |

### 9.2 Target 1 — Two solutions explained

```
Target:     [0.20,  0.20,  0.20,  pitch=1.571,  roll=0.000]

Solution 1: q = [1.6662,  1.3767,  1.5044,  0.0571,  0.2601]  rad
Solution 2: q = [1.6666,  1.3398,  1.5708,  0.0653,  0.2306]  rad
```

Both solutions are physically valid and geometrically distinct:

- **q1 ≈ 1.666 rad in both** — consistent with pointing toward (0.2, 0.2). The fixed `Rot_Z(π)` base offset
  shifts the effective waist angle, placing the arm at ~95° in joint space for a 45° world-frame direction.
- **q2 and q3 differ** (~0.037 and ~0.066 rad respectively) — this is the classic **elbow-up / elbow-down** pair.
  In solution 2, q3 = π/2 (fully extended elbow); solution 1 has a slightly relaxed elbow.
- q4 and q5 are small in both cases, absorbing the remaining orientation residual.

Finding two solutions here is the **expected behaviour** for a 5-DOF arm at a non-singular pose.

### 9.3 No-solution cases — geometric root causes

#### `[0.20, 0.10, 0.40, pitch=0.000, roll=1.571]`
Roll = π/2 requires the wrist to rotate 90° around the approach axis.
The wrist chain is `Rot_Y(+π/2) · Rot_Z(q4)` then `Rot_Y(-π/2) · Rot_Z(q5)`.
The two fixed `Rot_Y` rotations partially cancel, and with q4, q5 ∈ [−π/2, +π/2], the achievable
roll range at this position does not include π/2 once the position constraints are satisfied.

#### `[0.00, 0.00, 0.45, pitch=0.785, roll=0.785]`
The target lies on the robot's vertical (waist) axis — a **kinematic singularity for q1**.
At x = y = 0 the waist angle is undefined, but every choice of q1 then over-constrains the
remaining joints when a specific orientation (pitch ≠ 0, roll ≠ 0) must also be achieved.
The optimizer finds no consistent configuration.

#### `[0.00, 0.00, 0.07, pitch=3.141, roll=0.000]`
Two compounding reasons:
1. **The gripper centre cannot reach Z = 0.07 m** — the shoulder joint is already at Z ≈ 0.119 m
   (0.0165 + 0.1025 m above the world origin). Folding the arm below the shoulder mount requires
   q2 and q3 to swing past vertical, which needs values beyond their ±π/2 limits.
2. **pitch = π** means the end-effector must point straight down. No combination of q2–q5
   within ±π/2 can accumulate enough rotation to flip the approach vector by 180°.

#### `[0.00, 0.045, 0.45, pitch=0.785, roll=3.141]`
Roll = π (≈ 3.14 rad) requires a 180° rotation around the approach axis.
Each wrist joint is capped at ±π/2. There is no fixed roll offset in the kinematic chain
that could compensate — the maximum achievable wrist roll is well below π regardless of position.

### 9.4 Solver warning — `clipping to bounds`

```
RuntimeWarning: Values in x were outside bounds during a minimize step, clipping to bounds
```

This is **normal SLSQP behaviour**: the gradient step temporarily overshoots the joint limits
and is projected back onto the feasible set. The returned solutions respect all bounds.
It does not indicate a failed solve.
