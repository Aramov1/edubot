# EduBot Assignment — README

## Overview

This project implements the kinematics of a 5-DOF robotic arm (EduBot) and uses them to perform motion control tasks, both in simulation and on real hardware, using ROS2.

The implemented tasks are:

- **Workspace Visualisation** — visualise the robot's reachable workspace in RViz
- **Task 2 — Shape Following** — move the end-effector along geometric shapes (position control)
- **Task 3 — Velocity Controller** — track a trajectory using velocity-based closed-loop control
- **Task 4 — Pick and Place** — pick and place objects using LTPB trajectories

---

## Before Running Anything

Every terminal must have ROS2 sourced. From the project root:

```bash
source /opt/ros/<ros2-distro>/setup.bash
source ros_ws/install/setup.bash
```

After editing any Python file, rebuild the package:

```bash
cd ros_ws
colcon build && source install/setup.bash
```

---

## Workspace Visualisation

Visualises all positions reachable by the end-effector.

**Terminal 1** — launch RViz:
```bash
ros2 launch lerobot rviz.launch.py
```

**Terminal 2** — run the visualiser:
```bash
ros2 run assignment workspace_visualizer
```

In RViz, go to **Panels → Displays → Marker → Topic** and select `/workspace_markers`.

> If no points appear, stop and rerun `workspace_visualizer`.

---

## Task 2 — Shape Following (Position Control)

Moves the end-effector along a geometric path (circle, square, triangle, or half-ellipse) using inverse kinematics.

**Simulation:**
```bash
ros2 launch assignment sim_shape_follower.launch.py
```

**Real hardware:**
```bash
ros2 launch assignment hw_shape_follower.launch.py
```

### Changing the shape

Edit the `main()` function in:
```
ros_ws/src/assignment/assignment/simple_trajectory_follower.py
```

Available shapes and their parameters:

| Shape | Key parameters |
|---|---|
| `CircleTrajectory` | `center`, `radius`, `normal_axis`, `num_points` |
| `SquareTrajectory` | `center`, `side_length`, `normal_axis`, `num_points` |
| `TriangleTrajectory` | `center`, `side_length`, `normal_axis`, `num_points` |
| `HalfEllipseTrajectory` | `p1`, `p2`, `p3`, `segment`, `center`, `normal_axis`, `num_points` |

The `normal_axis` sets the drawing plane: `'z'` → XY, `'y'` → XZ, `'x'` → YZ.

Example — switching to a square:
```python
square = SquareTrajectory(center=(0, 0.25, 0), side_length=0.08, normal_axis='z', num_points=40)
all_poses.extend(square.get_poses())
```

Rebuild after editing.

---

## Task 3 — Velocity Controller

Tracks a trajectory using Closed-Loop Inverse Kinematics (CLIK) with feedforward velocity. The default trajectory is a vertical Z-axis oscillation.

**Simulation:**
```bash
ros2 launch assignment sim_vel_controller.launch.py
```

**Real hardware:**
```bash
ros2 launch assignment hw_vel_controller.launch.py
```

### Changing the trajectory

Edit the `send_z_trajectory()` call in the `main_client()` function of:
```
ros_ws/src/assignment/assignment/vel_trajectory_controller.py
```

```python
node.send_z_trajectory(x=0.0, y=0.3, z_center=0.2, amplitude=0.1,
                       v_max=0.05, a_max=0.1, n_cycles=5)
```

| Parameter | Description |
|---|---|
| `x`, `y` | Fixed horizontal position (m) |
| `z_center` | Centre of vertical oscillation (m) |
| `amplitude` | Half-range of oscillation — EE moves `z_center ± amplitude` |
| `v_max` | Cruise speed (m/s) |
| `a_max` | Acceleration during blend phases (m/s²) |
| `n_cycles` | Number of full up-down strokes |

Rebuild after editing.

---

## Task 4 — Pick and Place

Executes a full pick-and-place cycle (HOME → P1 → P4 → HOME → P4 → P1 → HOME) using LTPB trajectories and velocity-based CLIK control.

**Simulation:**
```bash
ros2 launch assignment sim_pick_and_place.launch.py
```

**Real hardware:**
```bash
ros2 launch assignment hw_pick_and_place.launch.py
```

### Changing pick/place positions

Edit the target coordinates at the top of:
```
ros_ws/src/assignment/assignment/LTPB_trajectory_action_client.py
```

```python
HOME = [0.15, 0.15, 0.05]   # Resting position      [x, y, z] in metres
P1   = [0.10, 0.15, 0.05]   # Pick/drop point 1
P4   = [0.20, 0.15, 0.05]   # Pick/drop point 2
```

Motion profile parameters (`v_max`, `a_max`) can be tuned in the `LTPB_Trajectory` constructor inside the same file.

Rebuild after editing.

---

## Kinematics Test Scripts (Tasks 1–3)

The `assignment_tests/` folder contains standalone Python scripts for testing the kinematics implementation outside of ROS2, one subfolder per task. Each task folder also contains a `media/` folder with images and videos of the robot results for that task.

```
assignment_tests/
├── robot_kinematics.py                      # Shared development copy
├── Task1_Mobility_and_Workspace/
│   ├── test_files/
│   │   ├── fw_kinematics.py                 # Forward kinematics test
│   │   └── save_symbolic_fw_kinematics.py   # Saves symbolic FK to .txt
│   └── media/                               # Images & videos for Task 1
├── Task2_Inverse_Kinematics/
│   ├── test_files/
│   │   └── inv_kinematics.py                # Inverse kinematics test
│   └── media/                               # Images & videos for Task 2
└── Task3_The_Jacobian/
    ├── test_files/
    │   ├── jacobian.py                      # Jacobian test
    │   └── save_symbolic_jacobian.py        # Saves symbolic Jacobian to .txt
    └── media/                               # Images & videos for Task 3
```

### How to run the test scripts

The test scripts interface with the RViz simulation, so first launch it in position control mode:

**Terminal 1:**
```bash
ros2 launch lerobot sim_position.launch.py
```

**Terminal 2** — navigate to the test script folder and run:
```bash
python3 <file_name>.py
```

To change which configurations are tested, add or remove entries from the `*_to_test` list at the top of each script.

> The `save_symbolic_*.py` scripts do not require the simulation — they just save the symbolic expressions to a `.txt` file for inspection.

---

## Repository Structure (Reference)

```
edubot/
├── assignment_tests/              # Standalone kinematics test scripts
│   ├── robot_kinematics.py        # Development copy — edit and test here
│   ├── Task1_Mobility_and_Workspace/
│   ├── Task2_Inverse_Kinematics/
│   └── Task3_The_Jacobian/
└── ros_ws/src/assignment/         # ROS2 package — deployed nodes
    └── assignment/
        └── robot_kinematics.py    # Tested copy — used by all ROS2 nodes
```

The intended workflow is: **implement and test in `assignment_tests/`**, then copy `robot_kinematics.py` into the ROS2 package once validated.
