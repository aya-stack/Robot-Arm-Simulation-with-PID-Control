# Robot-Arm-Simulation-with-PID-Control
interactive 2D robotic arm simulation

# 2D Robotic Arm Simulation with PID Control

![Robotic Arm Simulation](https://via.placeholder.com/800x400?text=2D+Robot+Arm+Simulation)  
*Visualization of the 2-link robotic arm reaching for target positions*

## Description

This project simulates a 2-degree-of-freedom (2DOF) robotic arm with PID control for smooth and accurate movement. The simulation includes:

- Real-time inverse kinematics calculations
- PID controller implementation for joint movement
- Interactive target setting via mouse clicks
- Visualization of joint angles and control errors over time
- ![image](https://github.com/user-attachments/assets/e7ca0691-829b-416d-bd74-dd89bbfe6287)


## Features

- **Forward & Inverse Kinematics**: Computes joint positions and required angles
- **PID Control System**: Tunable controller with proportional, integral, and derivative terms
- **Interactive Visualization**: Click anywhere to set new target positions
- **Trajectory Planning**: Smooth interpolation between current and target positions
- **Joint Limit Enforcement**: Prevents impossible configurations
- **Real-time Plots**: Visualize joint angles and controller performance

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/robot-arm-simulation.git
   cd robot-arm-simulation
   ```

2. Install required packages:
   ```bash
   pip install numpy matplotlib
   ```

## Usage

Run the simulation:
```bash
python robot_arm_simulation.py
```

**Interactions:**
- Click anywhere in the arm visualization area to set a new target position
- The arm will smoothly move to the new target using PID control

**Tuning Parameters:**
Modify these values in the code to change the arm's behavior:
```python
# PID gains
Kp = 1.5  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.05  # Derivative gain

# Max velocities
max_vel1 = 0.1  # Joint 1 max velocity (rad/frame)
max_vel2 = 0.1  # Joint 2 max velocity (rad/frame)
```

## Dependencies

- Python 3.x
- NumPy
- Matplotlib

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements.

## Acknowledgments

- Inspired by control theory and robotics coursework
- Uses Matplotlib for visualization
- PID control implementation based on classical control theory
