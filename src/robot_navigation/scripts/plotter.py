#!/usr/bin/env python3
"""
Real-time trajectory visualization.
Plots: (1) planned vs actual path with heading arrows
       (2) cross-track error over time
       (3) linear speed — planned vs actual
       (4) angular velocity (omega) profile
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import TwistStamped
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np


class PlotterNode(Node):
    def __init__(self):
        super().__init__('path_plotter')

        # Subscriptions
        self.create_subscription(Path,              '/planned_path',                    self._on_path,    10)
        self.create_subscription(Odometry,          '/odom',                             self._on_odom,    10)
        self.create_subscription(Bool,              '/navigation/goal_reached',          self._on_goal,    10)
        self.create_subscription(Float64MultiArray, '/visualization/velocity_profile',   self._on_vel,     10)
        self.create_subscription(Float64MultiArray, '/visualization/omega_profile',      self._on_omega,   10)
        self.create_subscription(TwistStamped,      '/cmd_vel',                          self._on_cmdvel,  10)

        # Data
        self.planned_path = None
        self.ref_theta, self.ref_v_arr, self.ref_omega_arr = [], [], []
        self.actual_x, self.actual_y = [], []
        self.error_t, self.error_cte = [], []
        self.vel_t, self.vel_ref = [], []
        self.omega_t, self.omega_ref = [], []
        self.cmd_t, self.cmd_v, self.cmd_w = [], [], []
        self.start_time = None
        self.goal_reached = False

        # 2×2 subplot layout
        plt.ion()
        self.fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(2, 2, figure=self.fig, hspace=0.4, wspace=0.3)
        self.ax_traj  = self.fig.add_subplot(gs[0, :])   # top: full width trajectory
        self.ax_cte   = self.fig.add_subplot(gs[1, 0])   # bottom-left: CTE
        self.ax_vel   = self.fig.add_subplot(gs[1, 1])   # bottom-right: speed

        self.create_timer(0.5, self._update_plot)
        self.get_logger().info('Plotter initialized.')

    # ---- Callbacks ----

    def _on_path(self, msg):
        self.planned_path = msg
        self.actual_x.clear(); self.actual_y.clear()
        self.error_t.clear();   self.error_cte.clear()
        self.vel_t.clear();     self.vel_ref.clear()
        self.omega_t.clear();   self.omega_ref.clear()
        self.cmd_t.clear();     self.cmd_v.clear(); self.cmd_w.clear()
        self.start_time = self.get_clock().now()
        self.goal_reached = False

        # Decode encoded kinematic state from orientation fields
        self.ref_theta.clear(); self.ref_v_arr.clear(); self.ref_omega_arr.clear()
        for ps in msg.poses:
            if abs(ps.pose.orientation.w - 1.0) < 1e-5:
                self.ref_theta.append(ps.pose.orientation.x)
                self.ref_v_arr.append(ps.pose.orientation.y)
                self.ref_omega_arr.append(ps.pose.orientation.z)
            else:
                self.ref_theta.append(0.0)
                self.ref_v_arr.append(0.0)
                self.ref_omega_arr.append(0.0)

    def _on_goal(self, msg):
        if msg.data:
            self.goal_reached = True
            self.get_logger().info('Goal reached — stopping plot updates.')

    def _on_vel(self, msg):
        d = np.array(msg.data)
        if d.size >= 2:
            self.vel_t   = d[0::2].tolist()
            self.vel_ref = d[1::2].tolist()

    def _on_omega(self, msg):
        d = np.array(msg.data)
        if d.size >= 2:
            self.omega_t   = d[0::2].tolist()
            self.omega_ref = d[1::2].tolist()

    def _on_cmdvel(self, msg):
        if self.start_time is None or self.goal_reached:
            return
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.cmd_t.append(t)
        self.cmd_v.append(msg.twist.linear.x)
        self.cmd_w.append(msg.twist.angular.z)

    def _on_odom(self, msg):
        if self.planned_path is None or self.start_time is None or self.goal_reached:
            return
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        self.actual_x.append(cx)
        self.actual_y.append(cy)

        # Cross-track error
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                         for p in self.planned_path.poses])
        dists_sq = np.sum((pts - np.array([cx, cy]))**2, axis=1)
        idx = int(np.argmin(dists_sq))
        if idx + 1 < len(pts):
            p1, p2 = pts[idx], pts[idx+1]
            seg_len = np.linalg.norm(p2 - p1)
            cte = np.abs(np.cross(p2-p1, np.array([cx,cy])-p1)) / seg_len if seg_len>1e-6 else 0.0
        else:
            cte = np.linalg.norm(pts[idx] - np.array([cx,cy]))

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.error_t.append(t)
        self.error_cte.append(cte)

    # ---- Plot update ----

    def _update_plot(self):
        if self.planned_path is None:
            return

        ref_x = [p.pose.position.x for p in self.planned_path.poses]
        ref_y = [p.pose.position.y for p in self.planned_path.poses]

        # --- Trajectory ---
        self.ax_traj.cla()
        self.ax_traj.plot(ref_x, ref_y, 'b-', lw=1.5, label='Reference path')
        if self.actual_x:
            self.ax_traj.plot(self.actual_x, self.actual_y, 'r--', lw=2, label='Actual path')

        # Heading arrows every ~10% of trajectory
        if self.ref_theta and len(self.ref_theta) == len(ref_x):
            step = max(1, len(ref_x) // 15)
            for i in range(0, len(ref_x), step):
                dx = 0.12 * np.cos(self.ref_theta[i])
                dy = 0.12 * np.sin(self.ref_theta[i])
                self.ax_traj.annotate('', xy=(ref_x[i]+dx, ref_y[i]+dy),
                                      xytext=(ref_x[i], ref_y[i]),
                                      arrowprops=dict(arrowstyle='->', color='green', lw=1.2))

        # Statistics
        if self.error_cte:
            mean_e = np.mean(self.error_cte)
            max_e  = np.max(self.error_cte)
            self.ax_traj.set_title(
                f'Trajectory Tracking   |   '
                f'Mean CTE: {mean_e:.4f}m   |   Max CTE: {max_e:.4f}m',
                fontsize=10)
        else:
            self.ax_traj.set_title('Trajectory Tracking')

        self.ax_traj.set_xlabel('X (m)'); self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.legend(loc='upper right'); self.ax_traj.grid(True)
        self.ax_traj.set_aspect('equal')

        # --- CTE ---
        self.ax_cte.cla()
        if self.error_t:
            self.ax_cte.plot(self.error_t, self.error_cte, 'r-', lw=1.5)
            self.ax_cte.axhline(np.mean(self.error_cte), color='orange',
                                ls='--', lw=1, label=f'Mean={np.mean(self.error_cte):.4f}m')
            self.ax_cte.legend(fontsize=8)
        self.ax_cte.set_title('Cross-Track Error vs Time')
        self.ax_cte.set_xlabel('Time (s)'); self.ax_cte.set_ylabel('CTE (m)')
        self.ax_cte.set_ylim(bottom=0); self.ax_cte.grid(True)

        # --- Velocity ---
        self.ax_vel.cla()
        if self.vel_t:
            self.ax_vel.plot(self.vel_t, self.vel_ref, 'b-', lw=1.5, label='Planned v')
        if self.cmd_t:
            self.ax_vel.plot(self.cmd_t, self.cmd_v, 'r--', lw=1.5, label='Commanded v')
        self.ax_vel.set_title('Linear Velocity Profile')
        self.ax_vel.set_xlabel('Time (s)'); self.ax_vel.set_ylabel('v (m/s)')
        self.ax_vel.legend(fontsize=8); self.ax_vel.grid(True); self.ax_vel.set_ylim(bottom=0)

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
        plt.ioff(); plt.show()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()