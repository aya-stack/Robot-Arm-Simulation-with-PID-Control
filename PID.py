import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# === Robot arm parameters ===
L1 = 2.0  # length of first link
L2 = 1.5  # length of second link

# Joint limits (radians)
theta1_min, theta1_max = -np.pi/2, np.pi/2
theta2_min, theta2_max = -np.pi, 0

# Max angular velocity per update (radians/frame)
max_vel1 = 0.1
max_vel2 = 0.1

# Initial joint angles
theta1 = 0.0
theta2 = 0.0

# Target point (start)
target = np.array([2.0, 1.0])

# PID gains (tune for different behaviors)
Kp = 1.5
Ki = 0.01
Kd = 0.05

# PID memory
e1_prev = 0
e2_prev = 0
int1 = 0
int2 = 0

# For plotting errors and angles over time
history_length = 200
theta1_history = []
theta2_history = []
e1_history = []
e2_history = []

# Trajectory interpolation parameters
interp_steps = 50
interp_index = 0
theta1_traj = []
theta2_traj = []

def inverse_kinematics(x, y):
    # Distance from origin to target
    dist = np.sqrt(x**2 + y**2)
    # Clamp target if unreachable
    if dist > (L1 + L2):
        x *= (L1 + L2) / dist
        y *= (L1 + L2) / dist

    # Law of cosines
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

    # Using atan2 for stable angle computation
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    # Adjust theta2 for elbow-down configuration (negative)
    theta2 = -theta2

    # Enforce joint limits on IK results
    theta1 = np.clip(theta1, theta1_min, theta1_max)
    theta2 = np.clip(theta2, theta2_min, theta2_max)

    return theta1, theta2

def generate_trajectory(start1, start2, goal1, goal2, steps):
    """Generate linear interpolation trajectory between two sets of joint angles."""
    traj1 = np.linspace(start1, goal1, steps)
    traj2 = np.linspace(start2, goal2, steps)
    return traj1, traj2

# Compute initial goal angles and trajectory
goal_theta1, goal_theta2 = inverse_kinematics(target[0], target[1])
theta1_traj, theta2_traj = generate_trajectory(theta1, theta2, goal_theta1, goal_theta2, interp_steps)
interp_index = 0

# === Matplotlib setup ===
fig, (ax_arm, ax_plots) = plt.subplots(2, 1, figsize=(7, 10))
plt.subplots_adjust(hspace=0.4)

# --- Arm plot ---
ax_arm.set_aspect('equal')
ax_arm.set_xlim(-4, 4)
ax_arm.set_ylim(-4, 4)
ax_arm.grid(True)
ax_arm.set_title("2D Robot Arm with PID & Interactive Target")

line, = ax_arm.plot([], [], 'o-', lw=4, color='teal')
target_dot, = ax_arm.plot([], [], 'rx', markersize=12)

# --- Angle and error plots ---
ax_plots.set_title("Joint Angles and PID Errors over Time")
ax_plots.set_xlim(0, history_length)
ax_plots.set_ylim(-3.5, 3.5)
ax_plots.grid(True)

angle1_line, = ax_plots.plot([], [], label='Theta1 (rad)', color='blue')
angle2_line, = ax_plots.plot([], [], label='Theta2 (rad)', color='green')
error1_line, = ax_plots.plot([], [], label='Error1', color='red', linestyle='dashed')
error2_line, = ax_plots.plot([], [], label='Error2', color='orange', linestyle='dashed')
ax_plots.legend(loc='upper right')

# === Update function for animation ===
def update(frame):
    global theta1, theta2, e1_prev, e2_prev, int1, int2, interp_index, theta1_traj, theta2_traj

    if interp_index < interp_steps:
        # Set intermediate goal from trajectory for smooth path
        goal_theta1 = theta1_traj[interp_index]
        goal_theta2 = theta2_traj[interp_index]
        interp_index += 1
    else:
        # Hold last goal angles
        goal_theta1 = theta1_traj[-1]
        goal_theta2 = theta2_traj[-1]

    # Errors
    e1 = goal_theta1 - theta1
    e2 = goal_theta2 - theta2

    # PID integrals
    int1 += e1
    int2 += e2

    # PID derivatives
    der1 = e1 - e1_prev
    der2 = e2 - e2_prev

    # PID control (desired velocity increment)
    vel1 = Kp * e1 + Ki * int1 + Kd * der1
    vel2 = Kp * e2 + Ki * int2 + Kd * der2

    # Limit max angular velocity for smoothness
    vel1 = np.clip(vel1, -max_vel1, max_vel1)
    vel2 = np.clip(vel2, -max_vel2, max_vel2)

    # Update joint angles with velocity limits
    theta1 += vel1
    theta2 += vel2

    # Enforce joint limits
    theta1 = np.clip(theta1, theta1_min, theta1_max)
    theta2 = np.clip(theta2, theta2_min, theta2_max)

    # Save errors for next derivative calculation
    e1_prev = e1
    e2_prev = e2

    # Forward kinematics to get points
    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    # Update arm plot
    line.set_data([x0, x1, x2], [y0, y1, y2])
    target_dot.set_data([target[0]], [target[1]])

    # Update history for plotting
    if len(theta1_history) >= history_length:
        theta1_history.pop(0)
        theta2_history.pop(0)
        e1_history.pop(0)
        e2_history.pop(0)

    theta1_history.append(theta1)
    theta2_history.append(theta2)
    e1_history.append(e1)
    e2_history.append(e2)

    # Update plots of angles and errors
    x_vals = np.arange(len(theta1_history))
    angle1_line.set_data(x_vals, theta1_history)
    angle2_line.set_data(x_vals, theta2_history)
    error1_line.set_data(x_vals, e1_history)
    error2_line.set_data(x_vals, e2_history)

    # Adjust y-limits dynamically if needed
    ax_plots.relim()
    ax_plots.autoscale_view()

    return line, target_dot, angle1_line, angle2_line, error1_line, error2_line

# === Interactive click to set new target ===
def on_click(event):
    global target, goal_theta1, goal_theta2, interp_index, theta1_traj, theta2_traj, int1, int2, e1_prev, e2_prev
    if event.inaxes != ax_arm:
        return
    target = np.array([event.xdata, event.ydata])
    goal_theta1, goal_theta2 = inverse_kinematics(target[0], target[1])
    # Generate new smooth trajectory from current angles to new goal
    theta1_traj, theta2_traj = generate_trajectory(theta1, theta2, goal_theta1, goal_theta2, interp_steps)
    interp_index = 0
    # Reset PID memory for smooth control start
    int1 = 0
    int2 = 0
    e1_prev = 0
    e2_prev = 0
    print(f"New target set: {target}")

fig.canvas.mpl_connect('button_press_event', on_click)

# Run animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 10000), interval=50, blit=True)
plt.show()
