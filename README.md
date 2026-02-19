# Path Smoothing and Trajectory Control

A ROS2 navigation stack for a TurtleBot3 differential-drive robot. Takes a list of waypoints and obstacles, generates a smooth collision-free trajectory, and executes it in Gazebo using an LPV-MPC controller.

For a detailed explanation of the algorithms and theory behind this implementation, see [THEORY.md](THEORY.md).

---

## System Overview

Four nodes work in a pipeline:

- **`waypoint_publisher_node`** — reads `waypoints.yaml` and `obstacles.yaml`, publishes to `/goal_waypoints`
- **`path_smoother_node`** — generates a Quintic Hermite spline trajectory with obstacle avoidance and trapezoidal velocity profile, publishes `/planned_path`
- **`trajectory_tracker_node`** — tracks the trajectory using LPV-MPC at 50Hz, publishes `/cmd_vel`
- **`plotter.py`** + **`mission_report.py`** — live plots and end-of-mission report

---

## System Requirements

**Operating System:** Ubuntu 24.04 (Noble)

**ROS2 Distribution:** Jazzy Jalisco

**Simulator:** Gazebo Harmonic (gz-sim 8), ships with ROS2 Jazzy

**Hardware:**
- Minimum 8GB RAM (16GB recommended — Gazebo + RViz are memory heavy)
- Dedicated GPU not required but speeds up Gazebo rendering
- If running headless (no display / SSH), Gazebo GUI and RViz will not launch — the simulation server and all navigation nodes run fine without them

**Python:** 3.12 (ships with Ubuntu 24.04)

**C++ standard:** C++17 or later

---

## Prerequisites

**ROS2 Jazzy + TurtleBot3 simulation repos:**
```bash
cd ~/your_ws/src
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/osrf/gazebo_models.git ~/gazebo_models
```

**Additional dependencies:**
```bash
sudo apt install ros-jazzy-osqp-vendor ros-jazzy-osqp-eigen python3-tk
pip install plotly --break-system-packages
```

---

## Build and Run

```bash
cd ~/your_ws
colcon build --packages-select robot_navigation --symlink-install
source install/setup.bash

# Validate math before running (no ROS needed)
./build/robot_navigation/test_spline
./build/robot_navigation/test_mpc

# Launch everything
./run.sh
```

---

## Configuration

All tuning lives in `config/navigation_params.yaml`.

**Waypoints** — edit `config/waypoints.yaml`:
```yaml
waypoints:
  - [0.0, 0.0]
  - [6.0, 0.0]
  - [6.0, 6.0]
  - [0.0, 6.0]
  - [0.0, 0.0]
```

**Obstacles** — edit `config/obstacles.yaml`:
```yaml
obstacles:
  centers:
    - [3.0, 3.0]
  radius: 0.35
```

**Key parameters:**

| Parameter | Default | Description |
|---|---|---|
| `velocity_profile` | `trapezoidal` | `trapezoidal` or `constant` |
| `max_velocity` | `0.22` | m/s |
| `corner_sharpness` | `0.25` | 0.0 = tight corners, 1.0 = wide |
| `mpc_horizon` | `20` | Prediction steps — increase for better curve anticipation |
| `mpc_q_x`, `mpc_q_y` | `150.0` | Position tracking weight |
| `mpc_q_theta` | `80.0` | Heading weight — reduce if oscillating on straights |
| `mpc_r_w` | `50.0` | Turn effort — increase to reduce oscillation |
| `lookahead_max` | `20` | Max horizon lookahead steps |

---

## Output

After the robot reaches the goal:
- `~/trajectory_plot.png` — planned vs actual path, CTE over time, velocity
- `~/mission_report.html` — interactive Plotly report (falls back to `mission_report.png` if plotly not installed)

---

## Real Robot

Replace the Gazebo launch with:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
The rest of the stack is unchanged — `/odom` and `/cmd_vel` topic names are identical. Re-tune MPC weights in `navigation_params.yaml` for real-world conditions.
