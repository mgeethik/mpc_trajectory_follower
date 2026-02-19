# **Path Smoothing and Trajectory Control Implementation**

This project is a complete navigation stack for a differential drive robot, implemented in ROS2 Jazzy. It takes a list of waypoints and obstacles, generates a smooth, collision-free trajectory, and executes it in the Gazebo simulator using a PID controller.

-----

## **1. Project Overview**

The system takes a coarse list of waypoints and a set of circular obstacles, and then:

1.  **Generates a smooth, collision-free path** that passes through the waypoints while navigating around the obstacles.
2.  **Creates a time-parameterized trajectory** along this path using configurable velocity profiles (Trapezoidal or Constant).
3.  **Executes the trajectory** in the Gazebo simulator using a PID controller to make the robot accurately follow the planned path.

The entire system is visualized in RViz, and real-time performance is graphed using a custom plotting script.

-----

## **2. System Architecture**

The system is designed with a modular, multi-node architecture, which is a standard practice in ROS2 for creating systems that are easy to debug and manage.

  * **`waypoint_publisher_node`**: This node acts as the mission server. It reads the waypoints and obstacles from their respective YAML files and publishes them for the rest of the system to use. It also publishes visualization markers (spheres and cylinders) for RViz.
  * **`path_smoother_node`**: This is the "brains" of the planner. It subscribes to the waypoints and obstacles and performs the heavy lifting of calculating a smooth, safe, and timed trajectory. It then publishes the final `/planned_path`.
  * **`trajectory_tracker_node`**: This is the robot's driver. It subscribes to the final `/planned_path` and the robot's current position (`/odom`). It continuously calculates and publishes velocity commands (`/cmd_vel`) to steer the robot along the path.
  * **`plotter.py`**: A Python-based visualization tool that subscribes to all relevant topics to generate real-time graphs of the robot's performance, including the trajectory tracking, error over time, and velocity profile.

-----

## **3. Core Algorithms Explained**

### **Path Smoothing: Catmull-Rom Spline**

To turn the jagged waypoint path into a smooth curve, we use an **interpolating Catmull-Rom Spline**.

  * **Why this algorithm?** Its most important feature is that it's an *interpolating* spline, meaning the final curve is **guaranteed to pass through every single waypoint**. This is critical, as waypoints often represent important locations (like the center of a hallway) that the robot must traverse.
  * **How it works:** It generates the curve piece by piece. To calculate the smooth segment between two points, $P_1$ and $P_2$, it uses a "sliding window" of four points: the point before ($P_0$) and the point after ($P_3$). These neighbors determine the tangent (or direction) of the curve as it passes through $P_1$ and $P_2$, ensuring a smooth transition between segments.
    The formula for a point on the curve between $P_1$ and $P_2$ is a cubic polynomial:
    $$
    C(t) = 0.5 \times \big[ (2P_1) + (-P_0 + P_2)t + (2P_0 - 5P_1 + 4P_2 - P_3)t^2 + (-P_0 + 3P_1 - 3P_2 + P_3)t^3 \big]
    $$
    where $t$ goes from 0 to 1.

### **Obstacle Avoidance: The "Check-Correct-Smooth" Method**

To handle obstacles, we use a robust three-phase approach.

1.  **Check:** First, we treat the path as simple straight lines between waypoints. We check each line segment to see if it intersects with any obstacle's safety radius.
2.  **Correct:** If a segment $(P_i, P_{i+1})$ is in collision, we find a detour around the obstacle by inserting a new, intermediate waypoint. This new waypoint is placed on the side of the obstacle, ensuring a clear path.
3.  **Smooth:** After the correction phase, we have a new set of waypoints that is guaranteed to be collision-free. We then feed this safe, corrected set of points into the Catmull-Rom spline algorithm. The spline naturally generates a smooth curve that follows the detours.

This method includes a **"Generate-then-Verify"** loop, where the final spline is checked for collisions one last time to guarantee that the curved path doesn't accidentally bulge into an obstacle.

### **Trajectory Generation: Velocity Profiles**

Once we have the final geometric shape of the path, we need to decide how fast the robot should move. This is handled by applying a velocity profile.

  * **Constant Velocity:** The robot moves at a single, constant speed from start to finish.
  * **Trapezoidal Velocity:** This is a more realistic profile. The robot accelerates at the start, cruises at a maximum speed, and then decelerates to a smooth stop at the end. This is achieved using standard equations of motion:
      * **Acceleration Phase:**
        $$
        d(t) = \frac{1}{2} a t^2
        $$

      * **Cruise Phase:**
        $$
        d(t) = d_{accel} + v_{max} \times t
        $$

        The planner uses these equations to calculate the exact point on the path the robot should be at for any given timestamp, creating the final trajectory.

### **Trajectory Tracking: PID Controller**

The `trajectory_tracker_node` uses a PID controller to make the robot follow the planned trajectory. The controller's output is based on two key errors calculated in real-time:

1.  **Cross-Track Error (CTE):** The perpendicular distance from the robot to the closest point on the path. This measures how far the robot is "off the line."
2.  **Heading Error ($\theta_{err}$):** The difference between the robot's current orientation and the path's orientation at the target point. This measures if the robot is "pointing the right way."

The steering command ($\omega$) is calculated using the standard PID formula, combining these errors:
$$\omega = (K_p \cdot \theta_{err}) + (K_i \cdot \int \theta_{err} dt) + (K_d \cdot \frac{d\theta_{err}}{dt}) + (K_{cte} \cdot CTE)$$
To improve stability, the robot's forward speed is automatically reduced when it needs to make sharp turns.

-----

## **4. Setup and How to Run**

### **Prerequisites**

  * Ubuntu 22.04 with ROS2 Jazzy installed.
  * Required ROS2 packages and Gazebo simulator:
    ```bash
    sudo apt update
    cd ~/ros2_ws/src
    git clone -b jazzy-devel [https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git](https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git)
    git clone -b jazzy-devel [https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)
    git clone -b jazzy-devel [https://github.com/ROBOTIS-GIT/DynamixelSDK.git](https://github.com/ROBOTIS-GIT/DynamixelSDK.git)
    ```
  * A local copy of the Gazebo models for simulation assets:
    ```bash
    git clone https://github.com/osrf/gazebo_models.git ~/gazebo_models
    ```

### **Running the Project**

1.  **Build the Package:**
    Navigate to your workspace root (`ros2_ws`) and build the package:
    ```bash
    colcon build --symlink-install
    ```
2.  **Run the Simulation:**
    In the workspace root, execute the provided runner script. This script sets up all the necessary environment variables and launches the entire system.
    ```bash
    ./run.sh
    ```

This will launch Gazebo, RViz, and all the custom C++ and Python nodes. The robot should start moving automatically.

-----

## **5. Configuration and Tuning**

You can easily change the behavior of the system by editing the configuration files in the `config` directory.
.sh
  * **`waypoints.yaml`**: Defines the sequence of points for the robot to follow.
  * **`obstacles.yaml`**: Defines the $[x, y]$ centers of the circular obstacles.
  * **`navigation_params.yaml`**: This is the main file for tuning.
      * **To tune the PID controller**, adjust the `kp_angular`, `ki_angular`, `kd_angular`, and `kp_cross_track` values under the `trajectory_tracker_node` section.
      * **To switch velocity profiles**, change the `velocity_profile` parameter under `path_smoother_node` to either `"constant"` or `"trapezoidal"`.

-----

## **6. Extension to a Real Robot**

To adapt this project for a real TurtleBot3, you would need to make the following changes:

1.  **Launch File:** You would not launch the Gazebo simulation. Instead, you would launch the official TurtleBot3 driver node, which communicates with the robot's hardware (`ros2 launch turtlebot3_bringup robot.launch.py`).
2.  **Odometry:** The `trajectory_tracker_node` would use the `/odom` topic published by the real robot's wheel encoders instead of the simulated odometry from Gazebo. No code change is needed for this, as the topic name is the same.
3.  **Controller Tuning:** The PID gains in `navigation_params.yaml` would need to be re-tuned. Real-world physics (wheel slip, battery level, floor surface) are different from the perfect physics in a simulator, so the controller will behave differently and require adjustments for optimal performance.