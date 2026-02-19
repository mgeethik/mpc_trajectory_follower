#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

class PlotterNode(Node):
    def __init__(self):
        super().__init__('path_plotter')
        self.planned_path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.goal_status_sub = self.create_subscription(
            Bool, '/navigation/goal_reached', self.goal_status_callback, 10)
        self.velocity_profile_sub = self.create_subscription(
            Float64MultiArray, '/visualization/velocity_profile', self.velocity_profile_callback, 10)

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)

        self.planned_path = None
        self.actual_path_x = []
        self.actual_path_y = []
        self.error_history = []
        self.time_history = []
        self.velocity_time = []
        self.velocity_data = []
        # **FIX**: New lists for actual velocity
        self.actual_velocity_time = []
        self.actual_velocity_data = []
        self.start_time = None
        self.goal_reached = False

        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))
        gs = GridSpec(2, 2, figure=self.fig)
        self.ax_trajectory = self.fig.add_subplot(gs[0, :])
        self.ax_error = self.fig.add_subplot(gs[1, 0])
        self.ax_velocity = self.fig.add_subplot(gs[1, 1])
        
        self.timer = self.create_timer(0.5, self.update_plot)
        self.get_logger().info('Plotter initialized with new layout.')

    def path_callback(self, msg):
        self.get_logger().info('Received planned path. Resetting plots...')
        self.planned_path = msg
        # Reset all data lists
        self.actual_path_x, self.actual_path_y = [], []
        self.error_history, self.time_history = [], []
        self.velocity_time, self.velocity_data = [], []
        self.actual_velocity_time, self.actual_velocity_data = [], []
        self.start_time = self.get_clock().now()
        self.goal_reached = False

    def goal_status_callback(self, msg):
        if msg.data:
            self.get_logger().info('Goal reached signal received. Stopping plot updates.')
            self.goal_reached = True

    def velocity_profile_callback(self, msg):
        data = np.array(msg.data)
        if data.size > 0:
            self.velocity_time = data[0::2]
            self.velocity_data = data[1::2]
            self.get_logger().info(f'Received velocity profile with {len(self.velocity_time)} points.')

    def cmd_vel_callback(self, msg):
        if self.start_time is None or self.goal_reached:
            return
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.actual_velocity_time.append(elapsed_time)
        self.actual_velocity_data.append(msg.twist.linear.x)

    def odom_callback(self, msg):
        if self.planned_path is None or self.start_time is None or self.goal_reached:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.actual_path_x.append(current_x)
        self.actual_path_y.append(current_y)
        
        planned_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.planned_path.poses])
        dists_sq = np.sum((planned_points - np.array([current_x, current_y]))**2, axis=1)
        closest_point_idx = np.argmin(dists_sq)

        if closest_point_idx < len(planned_points) - 1:
            p1 = planned_points[closest_point_idx]
            p2 = planned_points[closest_point_idx + 1]
        else:
            p1 = planned_points[closest_point_idx - 1]
            p2 = planned_points[closest_point_idx]

        if np.linalg.norm(p2 - p1) < 1e-6:
             cross_track_error = np.linalg.norm(np.array([current_x, current_y]) - p1)
        else:
            cross_track_error = np.abs(np.cross(p2-p1, np.array([current_x, current_y])-p1)) / np.linalg.norm(p2-p1)

        self.error_history.append(cross_track_error)
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.time_history.append(elapsed_time)

    def update_plot(self):
        if self.planned_path is None:
            return

        # --- Trajectory Plot ---
        self.ax_trajectory.cla()
        planned_x = [p.pose.position.x for p in self.planned_path.poses]
        planned_y = [p.pose.position.y for p in self.planned_path.poses]
        self.ax_trajectory.plot(planned_x, planned_y, 'b-', label='Planned Path')
        if self.actual_path_x:
            self.ax_trajectory.plot(self.actual_path_x, self.actual_path_y, 'r--', label='Actual Path')
        self.ax_trajectory.set_title('Robot Trajectory Tracking')
        self.ax_trajectory.set_xlabel('X (m)'); self.ax_trajectory.set_ylabel('Y (m)')
        self.ax_trajectory.legend(); self.ax_trajectory.grid(True); self.ax_trajectory.axis('equal')

        # --- Error Plot ---
        self.ax_error.cla()
        if self.time_history:
            self.ax_error.plot(self.time_history, self.error_history, 'r-')
        self.ax_error.set_title('Cross-Track Error vs. Time')
        self.ax_error.set_xlabel('Time (s)'); self.ax_error.set_ylabel('Error (m)')
        self.ax_error.grid(True); self.ax_error.set_ylim(bottom=0)

        # --- Velocity Plot ---
        self.ax_velocity.cla()
        #Plot actual velocity
        if len(self.actual_velocity_time) > 0:
            self.ax_velocity.plot(self.actual_velocity_time, self.actual_velocity_data, 'r--', label='Actual Velocity')
        self.ax_velocity.set_title('Velocity Profile')
        self.ax_velocity.set_xlabel('Time (s)'); self.ax_velocity.set_ylabel('Velocity (m/s)')
        self.ax_velocity.legend(); self.ax_velocity.grid(True); self.ax_velocity.set_ylim(bottom=0)
        
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

