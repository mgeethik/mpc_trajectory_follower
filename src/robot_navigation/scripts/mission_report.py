#!/usr/bin/env python3
"""
Mission Report Generator
Triggered when /navigation/goal_reached is published.
Saves an interactive HTML report to ~/mission_report.html
"""
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np
import matplotlib
matplotlib.use('Agg')

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec


class MissionReportNode(Node):
    def __init__(self):
        super().__init__('mission_report')

        self.create_subscription(Path,              '/planned_path',                   self._on_path,   10)
        self.create_subscription(Odometry,          '/odom',                            self._on_odom,   10)
        self.create_subscription(Bool,              '/navigation/goal_reached',         self._on_goal,   10)
        self.create_subscription(Float64MultiArray, '/visualization/velocity_profile',  self._on_vel,    10)
        self.create_subscription(Float64MultiArray, '/visualization/omega_profile',     self._on_omega,  10)
        self.create_subscription(TwistStamped,      '/cmd_vel',                         self._on_cmdvel, 10)

        self.planned_x, self.planned_y = [], []
        self.ref_theta, self.ref_v_plan, self.ref_omega = [], [], []
        self.actual_x, self.actual_y, self.actual_t = [], [], []
        self.cte_history = []
        self.vel_t_plan, self.vel_plan = [], []
        self.omega_t, self.omega_ref = [], []
        self.cmd_t, self.cmd_v, self.cmd_w = [], [], []
        self.start_time = None
        self.reported = False

        self.get_logger().info(
            f'Mission Report ready. '
            f'({"Plotly" if PLOTLY_AVAILABLE else "Matplotlib"} backend)')

    def _on_path(self, msg):
        self.planned_x = [p.pose.position.x for p in msg.poses]
        self.planned_y = [p.pose.position.y for p in msg.poses]
        self.ref_theta.clear(); self.ref_v_plan.clear(); self.ref_omega.clear()
        for p in msg.poses:
            if abs(p.pose.orientation.w - 1.0) < 1e-5:
                self.ref_theta.append(p.pose.orientation.x)
                self.ref_v_plan.append(p.pose.orientation.y)
                self.ref_omega.append(p.pose.orientation.z)
        self.actual_x.clear(); self.actual_y.clear(); self.actual_t.clear()
        self.cte_history.clear()
        self.cmd_t.clear(); self.cmd_v.clear(); self.cmd_w.clear()
        self.start_time = self.get_clock().now()
        self.reported = False

    def _on_goal(self, msg):
        if msg.data and not self.reported:
            self.reported = True
            self._generate_report()

    def _on_vel(self, msg):
        d = np.array(msg.data)
        if d.size >= 2: self.vel_t_plan = d[0::2].tolist(); self.vel_plan = d[1::2].tolist()

    def _on_omega(self, msg):
        d = np.array(msg.data)
        if d.size >= 2: self.omega_t = d[0::2].tolist(); self.omega_ref = d[1::2].tolist()

    def _on_cmdvel(self, msg):
        if self.start_time is None or self.reported: return
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.cmd_t.append(t); self.cmd_v.append(msg.twist.linear.x); self.cmd_w.append(msg.twist.angular.z)

    def _on_odom(self, msg):
        if self.start_time is None or self.reported: return
        cx, cy = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.actual_x.append(cx); self.actual_y.append(cy)
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.actual_t.append(t)
        if self.planned_x:
            pts = np.column_stack([self.planned_x, self.planned_y])
            dists_sq = np.sum((pts - np.array([cx,cy]))**2, axis=1)
            idx = int(np.argmin(dists_sq))
            if idx + 1 < len(pts):
                p1, p2 = pts[idx], pts[idx+1]
                seg = np.linalg.norm(p2-p1)
                cte = float(np.abs(np.cross(p2-p1, np.array([cx,cy])-p1)) / seg) if seg>1e-6 else 0.0
            else:
                cte = float(np.linalg.norm(pts[idx] - np.array([cx,cy])))
            self.cte_history.append(cte)

    def _stats(self):
        if not self.cte_history:
            return {'mean':0,'max':0,'std':0,'dur':0}
        return {
            'mean': float(np.mean(self.cte_history)),
            'max':  float(np.max(self.cte_history)),
            'std':  float(np.std(self.cte_history)),
            'dur':  float(self.actual_t[-1]) if self.actual_t else 0.0,
        }

    def _generate_report(self):
        stats = self._stats()
        self.get_logger().info(
            f'\n=== Mission Complete ===\n'
            f'  Mean CTE : {stats["mean"]:.4f} m\n'
            f'  Max CTE  : {stats["max"]:.4f} m\n'
            f'  Std CTE  : {stats["std"]:.4f} m\n'
            f'  Duration : {stats["dur"]:.1f} s\n'
            f'========================')

        save_path = os.path.expanduser('~/mission_report.html')

        if PLOTLY_AVAILABLE:
            self._save_plotly(stats, save_path)
        else:
            self._save_matplotlib(stats, save_path.replace('.html', '.png'))

    def _save_plotly(self, stats, path):
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=['Trajectory', 'Cross-Track Error', 'Linear Velocity', 'Angular Velocity (ω)'],
            specs=[[{'colspan': 2}, None], [{}, {}]])

        # --- Trajectory ---
        fig.add_trace(go.Scatter(x=self.planned_x, y=self.planned_y,
            mode='lines', name='Reference', line=dict(color='blue', width=2, dash='dash')), row=1, col=1)
        fig.add_trace(go.Scatter(x=self.actual_x, y=self.actual_y,
            mode='lines', name='Actual', line=dict(color='red', width=2.5)), row=1, col=1)

        # Heading arrows (every 10%)
        if self.ref_theta and len(self.ref_theta) == len(self.planned_x):
            step = max(1, len(self.planned_x) // 12)
            for i in range(0, len(self.planned_x), step):
                dx = 0.15 * np.cos(self.ref_theta[i])
                dy = 0.15 * np.sin(self.ref_theta[i])
                fig.add_annotation(
                    x=self.planned_x[i]+dx, y=self.planned_y[i]+dy,
                    ax=self.planned_x[i],   ay=self.planned_y[i],
                    xref='x', yref='y', axref='x', ayref='y',
                    showarrow=True, arrowhead=2, arrowcolor='green', arrowsize=1.2, row=1, col=1)

        # --- CTE ---
        if self.actual_t and self.cte_history:
            fig.add_trace(go.Scatter(x=self.actual_t, y=self.cte_history,
                mode='lines', name='CTE', line=dict(color='orange')), row=2, col=1)
            fig.add_hline(y=stats['mean'], line_dash='dot', line_color='red',
                          annotation_text=f'Mean={stats["mean"]:.4f}m', row=2, col=1)

        # --- Velocity ---
        if self.vel_t_plan:
            fig.add_trace(go.Scatter(x=self.vel_t_plan, y=self.vel_plan,
                mode='lines', name='Planned v', line=dict(color='blue', dash='dash')), row=2, col=2)
        if self.cmd_t:
            fig.add_trace(go.Scatter(x=self.cmd_t, y=self.cmd_v,
                mode='lines', name='Commanded v', line=dict(color='red')), row=2, col=2)

        fig.update_layout(
            title=(f'Mission Report — '
                   f'Mean CTE: {stats["mean"]:.4f}m | '
                   f'Max CTE: {stats["max"]:.4f}m | '
                   f'Std: {stats["std"]:.4f}m | '
                   f'Duration: {stats["dur"]:.1f}s'),
            template='plotly_white', height=800, width=1200)
        fig.update_yaxes(scaleanchor='x', scaleratio=1, row=1, col=1)

        fig.write_html(path)
        self.get_logger().info(f'Report saved: {path}')

    def _save_matplotlib(self, stats, path):
        fig = plt.figure(figsize=(14, 9))
        gs = GridSpec(2, 2, figure=fig, hspace=0.4)
        ax_traj = fig.add_subplot(gs[0, :])
        ax_cte  = fig.add_subplot(gs[1, 0])
        ax_vel  = fig.add_subplot(gs[1, 1])

        ax_traj.plot(self.planned_x, self.planned_y, 'b--', lw=1.5, label='Reference')
        ax_traj.plot(self.actual_x,  self.actual_y,  'r-',  lw=2,   label='Actual')
        ax_traj.set_title(f'Trajectory | Mean CTE:{stats["mean"]:.4f}m Max:{stats["max"]:.4f}m')
        ax_traj.set_aspect('equal'); ax_traj.grid(True); ax_traj.legend()

        if self.actual_t and self.cte_history:
            ax_cte.plot(self.actual_t, self.cte_history, 'r-')
            ax_cte.axhline(stats['mean'], color='orange', ls='--',
                           label=f'Mean={stats["mean"]:.4f}m')
        ax_cte.set_title('Cross-Track Error'); ax_cte.set_xlabel('Time (s)')
        ax_cte.set_ylabel('CTE (m)'); ax_cte.grid(True); ax_cte.legend()

        if self.vel_t_plan:
            ax_vel.plot(self.vel_t_plan, self.vel_plan, 'b--', label='Planned v')
        if self.cmd_t:
            ax_vel.plot(self.cmd_t, self.cmd_v, 'r-', label='Commanded v')
        ax_vel.set_title('Velocity'); ax_vel.set_xlabel('Time (s)')
        ax_vel.set_ylabel('v (m/s)'); ax_vel.grid(True); ax_vel.legend()

        plt.savefig(path, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Report saved: {path}')
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = MissionReportNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()