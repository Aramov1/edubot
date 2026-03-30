import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. Define Physical Constraints & Waypoints ---
v_max = 0.1  # Maximum cruise velocity
a_max = 0.2  # Maximum acceleration/deceleration
waypoint_x = [0, 0, 0.2, 0.2]  # X-coordinates of waypoints
waypoint_z = [0.05, 0.15, 0.15, 0.05]  # Z-coordinates of waypoints (Y-axis in 2D plot)
waypoint_y = [0.2, 0.2, 0, 0]  # Y-coordinates for visualization (not used in trajectory)

dx = waypoint_x[-1] - waypoint_x[0]  # Total horizontal distance (X-axis in this 2D plot)
dy = waypoint_y[-1] - waypoint_y[0]  # Total vertical distance (Y-axis in this 2D plot)

H = waypoint_z[1] - waypoint_z[0]    # Lift height (Y-axis in this 2D plot)
L = np.sqrt(dx**2 + dy**2)           # Traverse distance (X-axis in this 2D plot)

if L == 0:
    theta = 0.0
else:
    theta = np.arctan2(dy, dx)           # Angle of the traverse path

cos_theta = np.cos(theta)
sin_theta = np.sin(theta)

# Calculate blend parameters
t_b = v_max / a_max            # Time spent blending/accelerating
D = 0.5 * a_max * (t_b ** 2)   # Distance covered during blend

# Safety check (ensure path is long enough to reach v_max)
if H <= 2 * D or L <= 2 * D:
    raise ValueError("Height or Length is too short to reach v_max with this a_max.")

# --- 2. Define the Timeline ---
t0 = 0.0
t1 = t_b
t2 = t1 + (H - 2 * D) / v_max
t3 = t2 + t_b
t4 = t3 + (L - 2 * D) / v_max
t5 = t4 + t_b
t6 = t5 + (H - 2 * D) / v_max
t7 = t6 + t_b

# --- 3. Define the Decoupled Piecewise Functions ---
def get_local_x(t):
    if t < t2:
        return 0.0
    elif t < t3: # Corner 1 (Accel X)
        dt = t - t2
        return 0.5 * a_max * (dt ** 2)
    elif t < t4: # Traverse (Cruise X)
        dt = t - t3
        return D + v_max * dt
    elif t < t5: # Corner 2 (Decel X)
        dt = t - t4
        return (L - D) + (v_max * dt) - (0.5 * a_max * (dt ** 2))
    else:
        return L

def get_local_y(t):
    if t < t1:   # Lift (Accel Y)
        return 0.5 * a_max * (t ** 2)
    elif t < t2: # Lift (Cruise Y)
        dt = t - t1
        return D + v_max * dt
    elif t < t3: # Corner 1 (Decel Y)
        dt = t - t2
        return (H - D) + (v_max * dt) - (0.5 * a_max * (dt ** 2))
    elif t < t4: # Traverse (Wait Y)
        return H
    elif t < t5: # Corner 2 (Accel Y downwards)
        dt = t - t4
        return H - (0.5 * a_max * (dt ** 2))
    elif t < t6: # Place (Cruise Y downwards)
        dt = t - t5
        return (H - D) - (v_max * dt)
    elif t <= t7: # Place (Decel Y to stop)
        dt = t - t6
        return D - (v_max * dt) + (0.5 * a_max * (dt ** 2))
    else:
        return 0.0

# --- 4. Generate Data for Plotting ---
frames = 200
time_array = np.linspace(0, t7, frames)
x_path = [waypoint_x[0] + get_local_x(t)*cos_theta for t in time_array]
y_path = [waypoint_y[0] + get_local_x(t)*sin_theta for t in time_array]
z_path = [waypoint_z[0] + get_local_y(t) for t in time_array]

# --- 5. 3D Visualization & Animation ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the hard waypoints and the smooth path as a static background
ax.plot(waypoint_x, waypoint_y, waypoint_z, 'k--', alpha=0.5, label="Stop-and-Go Path")
ax.plot(x_path, y_path, z_path, 'b-', linewidth=1.5, alpha=0.5, label="Smooth Trajectory")

# Plot the start and end target points
ax.scatter([waypoint_x[0]], [waypoint_y[0]], [waypoint_z[0]], color='green', s=50, label="Start (Pick)")
ax.scatter([waypoint_x[-1]], [waypoint_y[-1]], [waypoint_z[-1]], color='red', s=50, label="End (Place)")

# Initialize the moving robot end-effector dot
robot_dot, = ax.plot([], [], [], 'ro', markersize=8, label="Robot TCP")

# Formatting the 3D axes
ax.set_title("3D Pick-and-Place Continuous Trajectory", fontsize=14)
ax.set_xlabel("Global X")
ax.set_ylabel("Global Y")
ax.set_zlabel("Global Z (Height)")

# Set dynamic limits based on your waypoints to keep the box proportional
ax.set_xlim(min(waypoint_x) - 0.1, max(waypoint_x) + 0.1)
ax.set_ylim(min(waypoint_y) - 0.1, max(waypoint_y) + 0.1)
ax.set_zlim(0, max(waypoint_z) + 0.1)
ax.legend(loc="upper left")

# --- Animation Functions ---
def init():
    # Initialize empty data for the dot
    robot_dot.set_data([], [])
    robot_dot.set_3d_properties([])
    return robot_dot,

def animate(i):
    # Update X and Y data
    robot_dot.set_data([x_path[i]], [y_path[i]])
    # Update Z data using the special 3D function
    robot_dot.set_3d_properties([z_path[i]])
    return robot_dot,

# Create the animation object (blit=False is usually safer for 3D Matplotlib)
ani = FuncAnimation(fig, animate, frames=frames, init_func=init, 
                    interval=int((t7/frames)*1000), blit=False, repeat=True)

plt.show()