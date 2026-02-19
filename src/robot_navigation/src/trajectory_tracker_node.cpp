#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "robot_navigation/mpc_controller.hpp"


#include <cmath>
#include <vector>
#include <limits>

class TrajectoryTrackerNode : public rclcpp::Node {
public:
    TrajectoryTrackerNode() : Node("trajectory_tracker_node")
    {
        // Parameters
        declare_parameter("max_linear_vel",  0.22);
        declare_parameter("max_angular_vel", 2.84);
        declare_parameter("goal_tolerance",  0.12);
        declare_parameter("mpc_horizon",     20);
        declare_parameter("mpc_dt",          0.02);
        declare_parameter("mpc_q_x",         150.0);
        declare_parameter("mpc_q_y",         150.0);
        declare_parameter("mpc_q_theta",      80.0);
        declare_parameter("mpc_r_v",          50.0);
        declare_parameter("mpc_r_w",          50.0);
        declare_parameter("mpc_terminal_scale", 5.0);
        declare_parameter("lookahead_min",    2);
        declare_parameter("lookahead_max",    10);
        declare_parameter("lookahead_gain",   20.0);

        get_parameter("max_linear_vel",      max_v_);
        get_parameter("max_angular_vel",     max_w_);
        get_parameter("goal_tolerance",      goal_tol_);
        get_parameter("mpc_horizon",         N_);
        get_parameter("mpc_dt",              mpc_dt_);
        get_parameter("lookahead_min",       lookahead_min_);
        get_parameter("lookahead_max",       lookahead_max_);
        get_parameter("lookahead_gain",      lookahead_gain_);

        // Configure MPC
        mpc_ = std::make_unique<LPVMPCController>(N_, mpc_dt_);
        mpc_->v_max          = max_v_;
        mpc_->w_max          = max_w_;
        get_parameter("mpc_q_x",            mpc_->q_x);
        get_parameter("mpc_q_y",            mpc_->q_y);
        get_parameter("mpc_q_theta",        mpc_->q_theta);
        get_parameter("mpc_r_v",            mpc_->r_v);
        get_parameter("mpc_r_w",            mpc_->r_w);
        get_parameter("mpc_terminal_scale", mpc_->terminal_scale);

        // Pub / Sub
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryTrackerNode::on_odom, this, std::placeholders::_1));
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&TrajectoryTrackerNode::on_path, this, std::placeholders::_1));
        cmd_pub_  = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        goal_pub_ = create_publisher<std_msgs::msg::Bool>("/navigation/goal_reached", 10);
        lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization/lookahead_target", 10);
        horizon_pub_ = create_publisher<nav_msgs::msg::Path>("/visualization/mpc_horizon", 10);

        // Control loop
        timer_ = create_wall_timer(
            std::chrono::duration<double>(mpc_dt_),
            std::bind(&TrajectoryTrackerNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "LPV-MPC Tracker ready. N=%d, dt=%.3fs", N_, mpc_dt_);
    }

private:
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q); double roll, pitch;
        m.getRPY(roll, pitch, theta_);

        v_meas_ = msg->twist.twist.linear.x;
        w_meas_ = msg->twist.twist.angular.z;
        odom_ok_ = true;
    }

    void on_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        px_.clear(); py_.clear(); pth_.clear(); pv_.clear(); pw_.clear();
        path_idx_ = 0;
        goal_reached_ = false;
        mpc_->reset();  // Reset warm-start when new path arrives

        for (const auto& ps : msg->poses) {
            px_.push_back(ps.pose.position.x);
            py_.push_back(ps.pose.position.y);

            bool encoded = (std::abs(ps.pose.orientation.w - 1.0) < 1e-5);
            if (encoded) {
                pth_.push_back(ps.pose.orientation.x);  // θ
                pv_.push_back(ps.pose.orientation.y);   // v
                pw_.push_back(ps.pose.orientation.z);   // ω
            } else {
                // Fallback: estimate heading from consecutive points
                pth_.push_back(0.0);
                pv_.push_back(max_v_);
                pw_.push_back(0.0);
            }
        }

        // Fill heading fallback for unencoded paths
        for (size_t i = 0; i < pth_.size(); ++i) {
            if (pth_[i] == 0.0 && i + 1 < pth_.size()) {
                pth_[i] = std::atan2(py_[i+1] - py_[i], px_[i+1] - px_[i]);
            }
        }

        path_ok_ = true;
        RCLCPP_INFO(get_logger(), "Trajectory received: %zu points.", px_.size());
    }

    // Bounded forward search for closest trajectory point.
    // Monotonically increases path_idx_ (robot can't go backward on the path).
    size_t find_closest(size_t start_from)
    {
        size_t best = start_from;
        double best_d = std::numeric_limits<double>::max();
        size_t search_end = std::min(start_from + 300, px_.size());

        for (size_t i = start_from; i < search_end; ++i) {
            double d = std::hypot(x_ - px_[i], y_ - py_[i]);
            if (d < best_d) { best_d = d; best = i; }
        }
        return best;
    }

    // Normalize angle to [-π, π]
    static double wrap(double a) {
        while (a >  M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

    void control_loop()
    {
        if (!odom_ok_ || !path_ok_ || px_.empty() || goal_reached_) return;

        // Check goal
        double d_goal = std::hypot(x_ - px_.back(), y_ - py_.back());
        if (d_goal < goal_tol_ && path_idx_ > px_.size() * 8 / 10) {
            RCLCPP_INFO(get_logger(), "Goal reached! dist=%.3fm", d_goal);
            goal_reached_ = true;
            send_zero();
            std_msgs::msg::Bool b; b.data = true; goal_pub_->publish(b);
            return;
        }

        // Update closest path index (forward only)
        size_t closest = find_closest(path_idx_);
        if (closest > path_idx_) path_idx_ = closest;

        // ---- Adaptive lookahead ----
        // Faster robot → look further ahead → smoother tracking, less lag on curves.
        // Slower robot → tighter lookahead → more responsive to path deviations.
        double speed_est = std::abs(v_meas_);
        int lookahead = static_cast<int>(
            std::clamp(lookahead_min_ + lookahead_gain_ * speed_est,
                       static_cast<double>(lookahead_min_),
                       static_cast<double>(lookahead_max_)));
        size_t target_idx = std::min(path_idx_ + static_cast<size_t>(lookahead),
                                     px_.size() - 1);

        // ---- Build MPC reference over horizon ----
        Eigen::Vector3d x0(x_, y_, theta_);
        std::vector<RefState> refs;
        refs.reserve(N_ + 1);

        double prev_theta = theta_;
        for (int k = 0; k <= N_; ++k) {
            size_t idx = std::min(target_idx + static_cast<size_t>(k), px_.size()-1);
            RefState rs;
            rs.x = px_[idx]; rs.y = py_[idx];

            // Unwrap heading angle (avoids ±π wrapping discontinuities in cost function)
            double raw_theta = pth_[idx];
            rs.theta = prev_theta + wrap(raw_theta - prev_theta);
            prev_theta = rs.theta;

            rs.v     = pv_[idx];
            rs.omega = pw_[idx];
            refs.push_back(rs);
        }

        // ---- Solve MPC ----
        auto sol = mpc_->solve(x0, refs);
        Eigen::Vector2d u = sol.u;

        // ---- Publish command ----
        auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd->header.stamp    = get_clock()->now();
        cmd->twist.linear.x  = u(0);
        cmd->twist.angular.z = u(1);
        cmd_pub_->publish(*cmd);

        // Publish lookahead target as a sphere marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = get_clock()->now();
        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = px_[target_idx];
        marker.pose.position.y = py_[target_idx];
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        lookahead_pub_->publish(marker);

        // Publish MPC predicted trajectory (actual optimizer output, starts from robot position)
        nav_msgs::msg::Path horizon_path;
        horizon_path.header.frame_id = "odom";
        horizon_path.header.stamp = get_clock()->now();

        for (const auto& state : sol.pred_traj) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = horizon_path.header;
            ps.pose.position.x = state(0);
            ps.pose.position.y = state(1);
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            horizon_path.poses.push_back(ps);
        }
        horizon_pub_->publish(horizon_path);

        v_meas_ = u(0);
        w_meas_ = u(1);
    }

    void send_zero() {
        for (int i = 0; i < 5; ++i) {
            auto c = std::make_unique<geometry_msgs::msg::TwistStamped>();
            c->header.stamp = get_clock()->now();
            cmd_pub_->publish(*c);
        }
    }

    // Members
    std::unique_ptr<LPVMPCController> mpc_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr horizon_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_=0, y_=0, theta_=0, v_meas_=0, w_meas_=0;
    bool odom_ok_ = false;

    std::vector<double> px_, py_, pth_, pv_, pw_;
    bool path_ok_ = false;
    size_t path_idx_ = 0;
    bool goal_reached_ = false;

    double max_v_, max_w_, goal_tol_, mpc_dt_;
    int N_, lookahead_min_, lookahead_max_;
    double lookahead_gain_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
    rclcpp::shutdown();
}