# LeRobot ROS2 Workspace Guide

This guide describes the full structure of the `ros_ws/` workspace for the **SO-ARM100 (LeRobot)** — a 5-DOF robotic arm with a gripper. It covers all packages, nodes, topics, services, launch files, and how all tools interact with each other.

---

## Table of Contents

1. [Workspace Overview](#1-workspace-overview)
2. [Package Descriptions & Roles](#2-package-descriptions--roles)
   - [robot_core](#21-robot_core--the-abstract-robot-base)
   - [robot_sim](#22-robot_sim--the-simulation-layer)
   - [feetech_cpp_lib](#23-feetech_cpp_lib--the-hardware-servo-driver)
   - [lerobot](#24-lerobot--the-robot-specific-implementation)
   - [controllers](#25-controllers--example-c-trajectory-controllers)
   - [python_controllers](#26-python_controllers--example-python-trajectory-controllers)
3. [Package Dependency Tree](#3-package-dependency-tree)
4. [Node Communication Reference](#4-node-communication-reference)
   - [Available Nodes](#41-available-nodes)
   - [Topics Reference](#42-topics-reference)
   - [Services Reference](#43-services-reference)
5. [Full Communication Architecture](#5-full-communication-architecture)
6. [Publishing & Subscribing — Impact Guide](#6-publishing--subscribing--impact-guide)
7. [Launch Files](#7-launch-files)
   - [rviz.launch.py](#71-rvizlaunchpy--standalone-visualization)
   - [sim_position.launch.py](#72-sim_positionlaunchpy--simulation-position-mode)
   - [sim_velocity.launch.py](#73-sim_velocitylaunchpy--simulation-velocity-mode)
   - [hw_position.launch.py](#74-hw_positionlaunchpy--hardware-position-mode)
   - [hw_velocity.launch.py](#75-hw_velocitylaunchpy--hardware-velocity-mode)
   - [lerobot_controller.launch.py](#76-lerobot_controllerlaunchpy--trajectory-controller)
8. [Tools Deep Dive](#8-tools-deep-dive)
   - [URDF File](#81-urdf-file)
   - [robot_state_publisher](#82-robot_state_publisher)
   - [joint_state_publisher_gui](#83-joint_state_publisher_gui)
   - [RViz2](#84-rviz2)
   - [TF2](#85-tf2)
9. [Configuration Files](#9-configuration-files)
10. [Development Workflow](#10-development-workflow)

---

## 1. Workspace Overview

```
ros_ws/
├── src/
│   ├── robot_core/          # Abstract base class for all robots
│   ├── robot_sim/           # Simulation layer (kinematic integrator)
│   ├── feetech_cpp_lib/     # Low-level servo driver (hardware only)
│   ├── lerobot/             # Robot-specific nodes, URDF, launch files
│   ├── controllers/         # Example C++ trajectory controllers
│   └── python_controllers/  # Example Python trajectory controllers
├── build/                   # Compiled artifacts (generated)
├── install/                 # Installed packages (generated)
└── log/                     # Build and run logs (generated)
```

The workspace has a **layered architecture**: abstract base → simulation/hardware implementation → robot-specific node → user controllers. Each layer depends only on the layer below it, which makes swapping hardware for simulation trivial.

---

## 2. Package Descriptions & Roles

### 2.1 `robot_core` — The Abstract Robot Base

**Language:** C++
**Role:** Defines the universal robot interface that every robot in this workspace must follow. Acts as the "contract" between the robot hardware/simulation and the user controllers.

**What it provides:**
- The `Robot` base class (inherits `rclcpp::Node`) with all ROS2 infrastructure pre-built
- Standard topic names for commands and state feedback
- A `SetMode` service to switch between Position and Velocity control at runtime
- Pure virtual methods that force subclasses to implement hardware-specific logic

**Key abstractions:**
```
Robot (abstract)
  ├── ROS infrastructure (subscribers, publishers, timers, service server)
  ├── Joint state management (q, qdot, q_des, qdot_des)
  └── Pure virtual methods → implemented by lerobot_hw or lerobot_sim
```

**Why it matters for development:** Any new controller you write only needs to publish to `/joint_cmds` and subscribe to `/joint_states`. It does not need to know whether a real robot or simulation is running. The `Robot` class handles all the dispatching internally.

**ROS Parameters (all nodes inherit these):**

| Parameter     | Default          | Description                                 |
|---------------|------------------|---------------------------------------------|
| `f`           | `100.0`          | Control loop frequency (Hz)                 |
| `pub_topic`   | `joint_states`   | Topic name for state output                 |
| `sub_topic`   | `joint_cmds`     | Topic name for position/velocity input      |
| `vel_sub_topic` | `joint_vel_cmds` | Topic name for velocity-only commands     |
| `mode`        | `position`       | Start mode: `"position"` or `"velocity"`   |

**Service Definition (`srv/SetMode.srv`):**
```
string mode     # "position" or "velocity"
---
bool success
```

---

### 2.2 `robot_sim` — The Simulation Layer

**Language:** C++
**Depends on:** `robot_core`
**Role:** Implements the `Robot` abstract interface using pure in-memory state. No physical hardware is needed. Simulates robot motion by integrating velocity commands over time (first-order kinematics).

**How simulation works:**

- **Position mode:** Joint positions are set directly from commanded values — no dynamics, no delays.
- **Velocity mode:** A timer runs at `sim_f` (25 Hz default) and integrates:
  ```
  q[i] = q[i] + qdot[i] * dt
  gripper = clamp(gripper + gripper_vel * dt, 0, 1)
  ```

**What it does NOT simulate:** gravity, inertia, joint limits enforcement, collisions, contact forces. It is a pure kinematic integrator.

**Why it matters for development:** Lets you develop and test controllers entirely without the physical robot. The published `/joint_states` are perfect (no noise, no latency), which is useful for algorithm development.

---

### 2.3 `feetech_cpp_lib` — The Hardware Servo Driver

**Language:** C++
**Role:** Communicates with Feetech STS-3215 smart servos over UART (USB-to-TTL serial). This is the lowest-level package — it directly reads and writes servo registers.

**Communication:** Uses `boost::asio` for serial I/O at 1 Mbps. The protocol is Dynamixel-compatible TTL.

**Key capabilities:**
- Position control (target angle in radians)
- Velocity control (target velocity in rad/s)
- Multi-turn tracking beyond the servo's native 0–4096 tick range
- Per-servo configuration: gear ratio, direction, home position, max speed
- Real-time feedback: position, velocity, current, temperature, PWM

**Hardware constants:**
```
4096 ticks per revolution
0.001534 rad/tick
Serial port: /dev/ttyUSB0
Baud rate: 1,000,000
Servo IDs: [11, 12, 13, 14, 15, 16]
```

**Why it matters for development:** You never interact with this library directly in normal use. It is only used internally by `lerobot_hw`. However, if you need to tune servo parameters (PID gains, max speed, direction), you will interact with `FeetechServo` methods through `lerobot_hw`.

---

### 2.4 `lerobot` — The Robot-Specific Implementation

**Language:** C++
**Depends on:** `robot_core`, `robot_sim`, `feetech_cpp_lib`
**Role:** The central package of the workspace. Contains everything specific to the SO-ARM100 robot: the hardware and simulation nodes, the URDF model, all launch files, the RViz config, and the path visualization node.

**Executables built by this package:**

| Executable      | Class          | Description                                      |
|-----------------|----------------|--------------------------------------------------|
| `lerobot_hw`    | `LeRobotHW`    | Hardware node — communicates with real servos    |
| `lerobot_sim`   | `LeRobotSim`   | Simulation node — runs in memory only            |
| `path_publisher`| `PathPublisher`| Visualizes end-effector trajectory in RViz       |

**Home configuration:**
```
Hardware:   {0°, -105°,  70°,  60°, 0°}
Simulation: {0°,  105°, -70°, -60°, 0°}  (sign-flipped for opposite kinematics convention)
```

**Hardware-specific parameters (`config/robot_hw.yaml`):**

| Parameter       | Value                          | Description                    |
|-----------------|--------------------------------|--------------------------------|
| `zero_positions`| `[1950, 1950, 1950, 2048, 2048, 2048]` | Home position in servo ticks |
| `ids`           | `[11, 12, 13, 14, 15, 16]`     | Servo IDs on the bus          |
| `gripper_open`  | `1.5708` rad (90°)             | Fully open gripper angle      |
| `gripper_closed`| `0.0` rad                      | Fully closed gripper angle    |
| `max_speed`     | `0.2` rad/s                    | Maximum joint speed           |

**`path_publisher` node:**
- Reads the `gripper_center` → `world` transform from TF every 100ms
- Maintains a rolling history of the last 25 poses
- Publishes to `/ee_path` as `nav_msgs/Path` for visualization in RViz

---

### 2.5 `controllers` — Example C++ Trajectory Controllers

**Language:** C++
**Depends on:** `rclcpp`, `trajectory_msgs`, Eigen3
**Role:** Provides ready-to-use example controllers that demonstrate how to send commands to the robot. These are **independent nodes** — they only communicate with the robot via the `/joint_cmds` topic.

**Executables:**

| Executable          | Mode     | Description                                      |
|---------------------|----------|--------------------------------------------------|
| `example_pos_traj`  | Position | Sinusoidal joint position trajectory (10s period)|
| `example_vel_traj`  | Velocity | Sinusoidal joint velocity trajectory (10s period)|

**Trajectory pattern (position):**
```
q[i](t) = home[i] + offset[i] * sin(2π * t / 10)
offsets: [0, 0.6, -0.4, -0.4, 0, 0] rad
gripper(t) = 0.5 * sin(2π * t / 10) + 0.5   (range 0–1)
```

**Trajectory pattern (velocity):**
```
qdot[i](t) = 0.025π * sin(2π * t / 10) rad/s
```

**Why it matters for development:** Use these as templates when writing your own controllers. They show the exact message format expected by `robot_core`.

---

### 2.6 `python_controllers` — Example Python Trajectory Controllers

**Language:** Python 3
**Depends on:** `rclpy`, `trajectory_msgs`, `numpy`
**Role:** Identical functionality to `controllers` but implemented in Python. Useful as a starting point for Python-based controller development.

**Entry points:**

| Entry Point        | Module & Function                              |
|--------------------|------------------------------------------------|
| `example_pos_traj` | `python_controllers.example_pos_traj:main`     |
| `example_vel_traj` | `python_controllers.example_vel_traj:main`     |

**Why it matters for development:** All Python controllers you write should follow this structure. Use `rclpy.Node`, publish to `/joint_cmds` with a `JointTrajectory` message, and you are immediately compatible with both `lerobot_sim` and `lerobot_hw`.

---

## 3. Package Dependency Tree

```
feetech_cpp_lib        (no ROS deps — pure serial driver)
       │
       └──> robot_core ──────────────────────┐
                │                            │
                └──> robot_sim               │
                          │                  │
                          └──> lerobot <─────┘
                               (lerobot_hw uses feetech_cpp_lib)
                               (lerobot_sim uses robot_sim)

controllers         ──> robot_core (for msg types only)
python_controllers  ──> robot_core (for msg types only)
```

**Reading the tree:**
- `robot_core` is the only package every other package depends on directly or transitively.
- `lerobot` is the top of the robot stack — it is what you actually launch.
- `controllers` and `python_controllers` are **sideways dependencies** — they depend on `robot_core` message types but are otherwise independent. They do not import `lerobot`.

---

## 4. Node Communication Reference

### 4.1 Available Nodes

| Node Name           | Package            | Executable         | Description                              |
|---------------------|--------------------|--------------------|------------------------------------------|
| `lerobot`           | `lerobot`          | `lerobot_hw`       | Hardware robot interface node            |
| `lerobot_sim`       | `lerobot`          | `lerobot_sim`      | Simulation robot interface node          |
| `path_publisher`    | `lerobot`          | `path_publisher`   | End-effector path visualization          |
| `robot_state_publisher` | (ROS2 built-in) | `robot_state_publisher` | FK computation + TF broadcaster    |
| `joint_state_publisher` | (ROS2 built-in) | `joint_state_publisher` | Publishes zero joint states        |
| `joint_state_publisher_gui` | (ROS2 built-in) | `joint_state_publisher_gui` | GUI sliders for joint control |
| `rviz2`             | (ROS2 built-in)   | `rviz2`            | 3D visualization                         |
| `example_pos_traj`  | `controllers`      | `example_pos_traj` | C++ sinusoidal position controller       |
| `example_vel_traj`  | `controllers`      | `example_vel_traj` | C++ sinusoidal velocity controller       |
| `example_pos_traj`  | `python_controllers` | `example_pos_traj` | Python sinusoidal position controller  |
| `example_vel_traj`  | `python_controllers` | `example_vel_traj` | Python sinusoidal velocity controller  |

---

### 4.2 Topics Reference

#### Topics Published by the Robot Node (`lerobot_hw` or `lerobot_sim`)

| Topic           | Message Type                       | QoS            | Rate     | Description                              |
|-----------------|------------------------------------|----------------|----------|------------------------------------------|
| `/joint_states` | `sensor_msgs/msg/JointState`       | SensorDataQoS  | 100 Hz   | Current joint positions, velocities, and effort for all 6 joints + gripper |

`/joint_states` message content:
```
header:
  stamp: <current time>
name:    ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Gripper"]
position: [q0, q1, q2, q3, q4, q_gripper]   (radians)
velocity: [v0, v1, v2, v3, v4, v_gripper]   (rad/s)
effort:   []
```

#### Topics Subscribed by the Robot Node

| Topic              | Message Type                           | QoS                    | Description                                                |
|--------------------|----------------------------------------|------------------------|------------------------------------------------------------|
| `/joint_cmds`      | `trajectory_msgs/msg/JointTrajectory`  | KeepLast(1), Reliable  | Position or velocity commands for all joints               |

`/joint_cmds` message format expected:
```
joint_names: ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
              "Wrist_Pitch", "Wrist_Roll", "Gripper"]
points:
  - positions:  [q0, q1, q2, q3, q4, q_gripper]   (in position mode)
    velocities: [v0, v1, v2, v3, v4, v_gripper]   (in velocity mode)
```
> Note: Only the **first point** in `points[]` is used. The robot immediately moves toward that command.

#### Topics Published by `path_publisher`

| Topic      | Message Type         | Rate     | Description                                           |
|------------|----------------------|----------|-------------------------------------------------------|
| `/ee_path` | `nav_msgs/msg/Path`  | 10 Hz    | Rolling history of last 25 end-effector poses (world frame) |

#### Topics Published by `robot_state_publisher` (ROS2 built-in)

| Topic                | Message Type              | Description                                     |
|----------------------|---------------------------|-------------------------------------------------|
| `/tf`                | `tf2_msgs/msg/TFMessage`  | Transform for every non-fixed joint in the URDF |
| `/tf_static`         | `tf2_msgs/msg/TFMessage`  | Transforms for fixed joints (published once)    |
| `/robot_description` | `std_msgs/msg/String`     | Full URDF XML string for RViz and other tools   |

---

### 4.3 Services Reference

| Service     | Package      | Type                       | Server Node | Description                         |
|-------------|--------------|----------------------------|-------------|-------------------------------------|
| `/set_mode` | `robot_core` | `robot_core/srv/SetMode`   | `lerobot` or `lerobot_sim` | Switch between position and velocity mode at runtime |

**Calling the service from terminal:**
```bash
ros2 service call /set_mode robot_core/srv/SetMode "{mode: 'velocity'}"
ros2 service call /set_mode robot_core/srv/SetMode "{mode: 'position'}"
```

---

## 5. Full Communication Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      YOUR CONTROLLER                            │
│  (controllers, python_controllers, or your custom node)         │
│                                                                 │
│   Publishes: /joint_cmds  (JointTrajectory)                     │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                   lerobot_hw  OR  lerobot_sim                   │
│                (inherits from robot_core::Robot)                │
│                                                                 │
│   Subscribes: /joint_cmds                                       │
│   Publishes:  /joint_states  (JointState, 100 Hz)              │
│   Service:    /set_mode                                         │
│                                                                 │
│   Hardware path:    FeetechServo → UART → /dev/ttyUSB0         │
│   Simulation path:  In-memory state integration                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /joint_states
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                   robot_state_publisher                         │
│                                                                 │
│   Subscribes: /joint_states                                     │
│   Reads:      /robot_description (URDF)                         │
│   Publishes:  /tf  (forward kinematics for every link)          │
│   Publishes:  /robot_description                                │
└──────────┬────────────────┬────────────────────────────────────┘
           │ /tf             │ /robot_description
           ▼                 ▼
┌──────────────┐    ┌────────────────────────────────────────────┐
│path_publisher│    │                  RViz2                     │
│              │    │                                            │
│ Reads TF     │    │  Displays: RobotModel (uses /robot_description + /tf)
│ Publishes    │    │  Displays: TF frames  (uses /tf)           │
│ /ee_path     │───▶│  Displays: Path       (uses /ee_path)      │
└──────────────┘    │  Displays: Grid, Axes                      │
                    └────────────────────────────────────────────┘
```

---

## 6. Publishing & Subscribing — Impact Guide

This section describes every topic you can interact with, what happens when you do, and what you should be careful about.

### `/joint_cmds` — Send Commands to the Robot

**Message type:** `trajectory_msgs/msg/JointTrajectory`
**Who reads it:** `lerobot_hw` or `lerobot_sim`

**In Position Mode:**
Publishing to this topic moves all joints to the specified angles. The robot (real or simulated) immediately starts moving toward the commanded position. The speed is bounded by `max_speed` (hardware) or the integration step (simulation).

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

msg = JointTrajectory()
msg.joint_names = ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
                   "Wrist_Pitch", "Wrist_Roll", "Gripper"]
pt = JointTrajectoryPoint()
pt.positions = [0.0, 1.0472, -0.8727, 0.8727, 0.0, 0.5]  # radians
msg.points = [pt]
publisher.publish(msg)
```

**In Velocity Mode:**
Publishing velocities causes the robot to move continuously. If you stop publishing, the last commanded velocity continues until you send zeros. Always send a zero-velocity command before stopping a velocity controller.

```python
pt.velocities = [0.0, 0.1, -0.1, 0.0, 0.0, 0.0]  # rad/s
```

**Important joint order and limits:**

| Index | Joint Name          | Limits (rad)       | Note                         |
|-------|---------------------|--------------------|------------------------------|
| 0     | `Shoulder_Rotation` | `[-2.0, 2.0]`      | Base rotation                |
| 1     | `Shoulder_Pitch`    | `[-1.57, 1.57]`    | Shoulder up/down             |
| 2     | `Elbow`             | `[-1.58, 1.58]`    |                              |
| 3     | `Wrist_Pitch`       | `[-1.57, 1.57]`    |                              |
| 4     | `Wrist_Roll`        | `[-3.14159, 3.14159]` | Full rotation             |
| 5     | `Gripper`           | `[0.0, 1.57]` (HW) | 0 = closed, 1.57 = open     |

> **Safety (Hardware):** The `max_speed` parameter (`0.2 rad/s`) limits the servo speed. Never command large position jumps on hardware — the robot will move at max speed toward the target with no trajectory interpolation.

---

### `/joint_states` — Read Robot State

**Message type:** `sensor_msgs/msg/JointState`
**Who publishes it:** `lerobot_hw` or `lerobot_sim`

Subscribe to this topic to read the current joint configuration. This is the primary feedback channel. On hardware, values come from the real servo encoders. In simulation, values are the integrated state.

```python
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    positions = msg.position  # list of 6 floats (rad)
    velocities = msg.velocity # list of 6 floats (rad/s)
    names = msg.name          # joint name for each value
```

Publishing to `/joint_states` yourself is **not recommended** — it would conflict with `robot_state_publisher`'s FK computation and corrupt the TF tree.

---

### `/ee_path` — Read End-Effector History

**Message type:** `nav_msgs/msg/Path`
**Who publishes it:** `path_publisher` node (must be launched explicitly)

Subscribe to visualize or record the Cartesian trajectory of the gripper center. Each pose in the path is in the `world` frame. The path stores the last 25 poses (configurable via the `path_length` parameter).

---

### `/tf` and `/tf_static` — Read Cartesian Poses (FK result)

**Who publishes it:** `robot_state_publisher`

Do not publish to these topics directly. Instead, use `tf2_ros.TransformListener` to read any link pose:

```python
import tf2_ros
from rclpy.time import Time

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

# Get end-effector pose in world frame
t = tf_buffer.lookup_transform('world', 'gripper_center', Time())
x, y, z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
```

Available TF frames (from the URDF): `world`, `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, `gripper_center`, `jaw`

---

## 7. Launch Files

### 7.1 `rviz.launch.py` — Standalone Visualization

**Package:** `lerobot`
**Command:** `ros2 launch lerobot rviz.launch.py`

**Purpose:** Visualize the robot model with manual joint sliders. No controller or simulation node is started. This is used for inspecting the URDF, verifying joint transforms, or testing RViz display configuration.

**Nodes launched:**

| Node                      | Condition        | Description                                      |
|---------------------------|------------------|--------------------------------------------------|
| `joint_state_publisher_gui` | `gui:=True` (default) | GUI sliders — one per joint — to manually control the visualization |
| `joint_state_publisher`   | `gui:=False`     | Publishes all-zero joint states silently (no GUI)|
| `robot_state_publisher`   | always           | Reads URDF + joint states → publishes TF         |
| `rviz2`                   | `use_rviz:=True` (default) | 3D visualizer                        |

**Launch arguments:**

| Argument          | Default | Description                                    |
|-------------------|---------|------------------------------------------------|
| `gui`             | `True`  | Show joint slider GUI                          |
| `use_rviz`        | `True`  | Launch RViz                                    |
| `use_robot_state_pub` | `True` | Launch robot_state_publisher               |
| `use_sim_time`    | `True`  | Use ROS simulated clock                        |
| `urdf_model`      | (auto)  | Path to URDF file                              |
| `rviz_config_file`| (auto)  | Path to `.rviz` config                         |

**Tool interaction:**
```
joint_state_publisher_gui  ──(sliders)──>  /joint_states
                                                │
                                                ▼
                              robot_state_publisher
                              (reads URDF, computes FK)
                                                │
                                           /tf, /robot_description
                                                │
                                                ▼
                                            RViz2
                                   (renders robot model)
```

---

### 7.2 `sim_position.launch.py` — Simulation Position Mode

**Package:** `lerobot`
**Command:** `ros2 launch lerobot sim_position.launch.py`

**Purpose:** Run the robot simulator in position control mode with full RViz visualization. This is the primary development environment for testing position-based controllers without hardware.

**Nodes launched:**

| Node                  | Description                                          |
|-----------------------|------------------------------------------------------|
| `lerobot_sim`         | Simulation node, `mode: position`                    |
| `robot_state_publisher` | FK computation from URDF + joint states            |
| `rviz2`               | 3D visualization (uses `rviz_basic_settings.rviz`)   |

**Launch arguments:** Same as `rviz.launch.py` minus the `gui` argument.

**Workflow:**
```
Your controller  →  /joint_cmds (positions)
                         │
                    lerobot_sim (direct state assignment)
                         │
                    /joint_states (100 Hz)
                         │
              robot_state_publisher
                         │
                    /tf (all link frames)
                         │
                       RViz2
```

> **Note:** The `path_publisher` is commented out in this launch file. To enable the end-effector path visualization, uncomment the `start_path_pub` lines.

---

### 7.3 `sim_velocity.launch.py` — Simulation Velocity Mode

**Package:** `lerobot`
**Command:** `ros2 launch lerobot sim_velocity.launch.py`

**Purpose:** Identical to `sim_position.launch.py` but starts `lerobot_sim` in velocity control mode. Use this when testing velocity controllers. The simulation will integrate velocity commands into positions.

**Key difference from position mode:**

| Aspect         | Position Mode                  | Velocity Mode                          |
|----------------|--------------------------------|----------------------------------------|
| `joint_cmds`   | Use `points[0].positions`      | Use `points[0].velocities`             |
| Robot response | Moves directly to target angle | Continuously integrates velocity × dt  |
| Stop behavior  | Holds last commanded position  | Continues moving at last velocity      |

---

### 7.4 `hw_position.launch.py` — Hardware Position Mode

**Package:** `lerobot`
**Command:** `ros2 launch lerobot hw_position.launch.py`

**Purpose:** Connect to the real SO-ARM100 robot and control it in position mode. Loads hardware parameters from `config/robot_hw.yaml`.

**Nodes launched:**

| Node       | Description                                                |
|------------|------------------------------------------------------------|
| `lerobot`  | Hardware node (`lerobot_hw`), `mode: position`, reads servo feedback from `/dev/ttyUSB0` |

> **Important:** This launch file does **not** start `robot_state_publisher` or RViz. To visualize hardware state in RViz, you must run `rviz.launch.py` (or start `robot_state_publisher` separately) in a second terminal after launching this.

**Hardware requirements:**
- USB-to-TTL adapter connected at `/dev/ttyUSB0`
- Robot powered on, servo IDs 11–16 responding
- User in `dialout` group: `sudo usermod -a -G dialout $USER`

---

### 7.5 `hw_velocity.launch.py` — Hardware Velocity Mode

**Package:** `lerobot`
**Command:** `ros2 launch lerobot hw_velocity.launch.py`

**Purpose:** Same as `hw_position.launch.py` but starts the hardware node in velocity mode. Servos will continuously move in the commanded direction until stopped.

> **Safety warning:** Always have a way to emergency-stop the robot when using velocity mode on hardware. A controller crash or topic publisher crash will leave the last velocity command active.

---

### 7.6 `lerobot_controller.launch.py` — Trajectory Controller

**Package:** `controllers`
**Command:** `ros2 launch controllers lerobot_controller.launch.py`

**Purpose:** Launch the example sinusoidal position trajectory controller. This node publishes to `/joint_cmds` and is designed to run alongside any of the robot launch files above.

**Nodes launched:**

| Node                  | Description                                    |
|-----------------------|------------------------------------------------|
| `example_raj_lerobot` | Example position trajectory (`example_pos_traj` executable) |

**Parameters loaded from:** `controllers/config/lerobot_params.yaml`
```yaml
home: [0.0, 1.0472, -0.8727, 0.8727, 0.0]  # radians
```

**Typical combined usage:**
```bash
# Terminal 1: Start simulator
ros2 launch lerobot sim_position.launch.py

# Terminal 2: Start trajectory controller
ros2 launch controllers lerobot_controller.launch.py
```

---

## 8. Tools Deep Dive

### 8.1 URDF File

**Location:** `lerobot/urdf/lerobot.urdf`
**Robot name:** `so_arm100`

The URDF (Unified Robot Description Format) is an XML file that fully defines the robot's physical structure. It is the central piece of documentation — everything that `robot_state_publisher`, RViz, and future IK solvers use comes from this file.

**What the URDF defines:**

1. **Links** — Physical rigid bodies (base, shoulder, upper_arm, lower_arm, wrist, gripper, gripper_center, jaw)
   - Mass and inertia tensors (for dynamics simulation if needed)
   - Visual geometry: STL mesh files from `lerobot/meshes/`
   - Collision geometry: same STL meshes

2. **Joints** — Connections between links:

| Joint Name           | Type     | Parent → Child          | Axis  | Limits (rad)        |
|----------------------|----------|-------------------------|-------|---------------------|
| `world_joint`        | fixed    | world → base            | —     | —                   |
| `Shoulder_Rotation`  | revolute | base → shoulder         | Z     | `[-2.0, 2.0]`       |
| `Shoulder_Pitch`     | revolute | shoulder → upper_arm    | Z     | `[-1.57, 1.57]`     |
| `Elbow`              | revolute | upper_arm → lower_arm   | Z     | `[-1.58, 1.58]`     |
| `Wrist_Pitch`        | revolute | lower_arm → wrist       | Z     | `[-1.57, 1.57]`     |
| `Wrist_Roll`         | revolute | wrist → gripper         | Z     | `[-3.14, 3.14]`     |
| `Gripper_Center_Fixed` | fixed  | gripper → gripper_center | —    | —                   |
| `Gripper`            | revolute | gripper → jaw           | Z     | `[-0.2, 2.0]`       |

3. **Materials** — Visual colors:
   - `3d_printed`: pink/magenta (`rgba 0.949 0.106 0.698 1.0`)
   - `sts3215`: dark gray (`rgba 0.1 0.1 0.1 1.0`)

**Relationship between URDF joints and servo IDs:**
```
Servo 11 → Shoulder_Rotation
Servo 12 → Shoulder_Pitch
Servo 13 → Elbow
Servo 14 → Wrist_Pitch
Servo 15 → Wrist_Roll
Servo 16 → Gripper
```

**How to use the URDF for development:**
If you add a new end-effector, tool, or sensor, you will modify the URDF to add new links and joints. Any new link automatically gets a TF frame broadcast by `robot_state_publisher`, making it immediately available for use in controllers and visualization.

---

### 8.2 `robot_state_publisher`

**Package:** `robot_state_publisher` (ROS2 built-in)
**Role:** Bridges joint angles to 3D Cartesian space. It is the node that performs forward kinematics.

**How it works:**
1. At startup, reads the URDF from the `robot_description` parameter
2. Subscribes to `/joint_states`
3. For every incoming joint state message, applies the kinematic chain from the URDF
4. Computes the 6-DOF transform (position + orientation) of every link relative to every parent
5. Broadcasts these transforms to `/tf` and `/tf_static`

**Why this matters:** This node is what makes `/tf` queries possible. Any node that calls `tf_buffer.lookup_transform('world', 'gripper_center', ...)` is reading the result of the FK computed here.

**Required to run when using:**
- RViz with a RobotModel display
- `path_publisher`
- Any node that reads TF frames
- IK solvers that need FK feedback

---

### 8.3 `joint_state_publisher_gui`

**Package:** `joint_state_publisher_gui` (ROS2 built-in)
**Role:** Provides a graphical window with one slider per non-fixed joint. Moving a slider publishes the new joint state to `/joint_states`.

**Used in:** `rviz.launch.py` only (when `gui:=True`)

**Interaction diagram:**
```
User moves slider  →  joint_state_publisher_gui  →  /joint_states
                                                          │
                                              robot_state_publisher
                                                          │
                                                      /tf update
                                                          │
                                                    RViz re-renders
```

**Important:** When using a simulation or hardware launch, this node must NOT be running — both would publish to `/joint_states` simultaneously, causing conflicts. It is only intended for pure visualization without a robot node.

---

### 8.4 RViz2

**Package:** `rviz2` (ROS2 built-in)
**Config file:** `lerobot/rviz/rviz_basic_settings.rviz`
**Role:** 3D visualization tool. Displays robot model, TF frames, trajectories, and any other visual topics.

**Active displays configured in `rviz_basic_settings.rviz`:**

| Display Name  | Type         | Data Source              | Description                              |
|---------------|--------------|--------------------------|------------------------------------------|
| Grid          | Grid         | (static)                 | Reference XY floor grid                 |
| RobotModel    | RobotModel   | `/robot_description`     | 3D mesh model of the robot               |
| TF            | TF           | `/tf`                    | Coordinate frames as colored axes        |
| Path          | Path         | `/ee_path`               | End-effector Cartesian trajectory        |
| Marker        | Marker       | (custom topic)           | Generic visual markers                   |

**RobotModel display detail:**
- Reads the URDF from `/robot_description` to know mesh file paths and link structure
- Reads `/tf` to know where to position and orient each mesh in 3D space
- Renders each link mesh at its current TF-computed pose

**Fixed frame:** `world` — all visualizations are rendered relative to this frame.

**How RViz, URDF, TF, and joint states all connect:**
```
URDF file ──────────────────────────────────────────────┐
(mesh paths, link structure)                             │
                                                         ▼
/joint_states ──> robot_state_publisher ──> /tf ──> RViz RobotModel
                  (applies URDF kinematics)         (positions meshes)
                                                         │
                          /robot_description ────────────┘
                          (URDF string, tells RViz which meshes to load)
```

---

### 8.5 TF2

**Role:** The transform library. Maintains a tree of coordinate frame relationships over time. Every link in the URDF becomes a named frame in TF2.

**Frame tree for this robot:**
```
world (fixed)
  └── base (fixed, 180° rotation from world)
        └── shoulder (revolves: Shoulder_Rotation)
              └── upper_arm (revolves: Shoulder_Pitch)
                    └── lower_arm (revolves: Elbow)
                          └── wrist (revolves: Wrist_Pitch)
                                └── gripper (revolves: Wrist_Roll)
                                      ├── gripper_center (fixed, +0.075m Z)
                                      └── jaw (revolves: Gripper)
```

**Querying TF from terminal:**
```bash
# Print the current transform between two frames
ros2 run tf2_ros tf2_echo world gripper_center

# Generate a PDF of the full TF tree
ros2 run tf2_tools view_frames
```

**Querying TF from Python:**
```python
import tf2_ros, rclpy
from rclpy.time import Time

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

# Forward kinematics: world -> gripper_center
t = tf_buffer.lookup_transform('world', 'gripper_center', Time())
```

---

## 9. Configuration Files

### `lerobot/config/robot_hw.yaml`

Used by `hw_position.launch.py` and `hw_velocity.launch.py`. Contains all hardware-specific tuning values.

```yaml
lerobot:
  ros__parameters:
    zero_positions: [1950, 1950, 1950, 2048, 2048, 2048]  # Home in servo ticks
    ids: [11, 12, 13, 14, 15, 16]                          # Servo IDs on TTL bus
    gripper_open: 1.57079632679                            # 90° = fully open
    gripper_closed: 0.0                                    # 0° = fully closed
    max_speed: 0.2                                         # rad/s speed limit
```

**Modifying this file:**
- Change `zero_positions` if the robot's physical zero position has changed (e.g., after reassembly)
- Change `max_speed` to allow faster or safer movement
- `ids` should match the servo IDs programmed onto the physical servo hardware

### `controllers/config/lerobot_params.yaml`

Used by `lerobot_controller.launch.py`. Sets the home configuration for the example C++ trajectory controller.

```yaml
example_traj_lerobot:
  ros__parameters:
    home: [0.0, 1.0472, -0.8727, 0.8727, 0.0]  # radians: ~[0°, 60°, -50°, 50°, 0°]
```

### `lerobot/rviz/rviz_basic_settings.rviz`

RViz2 configuration file. Defines which displays are active, their data sources, and the camera/view settings. Can be edited directly in RViz (File → Save Config) to persist your preferred view.

---

## 10. Development Workflow

### Starting a New Python Controller

1. Create your node in `python_controllers/python_controllers/my_controller.py`
2. Register it in `python_controllers/setup.py` under `console_scripts`
3. Rebuild: `cd ~/TU_Delft/edubot/ros_ws && colcon build --packages-select python_controllers`
4. Source: `source install/setup.bash`
5. Launch sim: `ros2 launch lerobot sim_position.launch.py`
6. Run controller: `ros2 run python_controllers my_controller`

### Switching Between Simulation and Hardware

Your controller code does not need to change. Simply change which launch file you run:

| Environment         | Launch Command                                      |
|---------------------|-----------------------------------------------------|
| Simulation Position | `ros2 launch lerobot sim_position.launch.py`        |
| Simulation Velocity | `ros2 launch lerobot sim_velocity.launch.py`        |
| Hardware Position   | `ros2 launch lerobot hw_position.launch.py`         |
| Hardware Velocity   | `ros2 launch lerobot hw_velocity.launch.py`         |

### Switching Control Mode at Runtime

Without restarting any node:
```bash
ros2 service call /set_mode robot_core/srv/SetMode "{mode: 'velocity'}"
ros2 service call /set_mode robot_core/srv/SetMode "{mode: 'position'}"
```

### Monitoring the System

```bash
# List all active topics
ros2 topic list

# Watch joint states at full rate
ros2 topic echo /joint_states

# Measure actual publish rate
ros2 topic hz /joint_states

# List all active nodes
ros2 node list

# Inspect what a node publishes/subscribes
ros2 node info /lerobot_sim

# List available services
ros2 service list
```

### Build Order

When building from scratch, `colcon` handles dependency ordering automatically:
```bash
cd ~/TU_Delft/edubot/ros_ws
colcon build
source install/setup.bash
```

To rebuild only one package after changes:
```bash
colcon build --packages-select lerobot
source install/setup.bash
```
