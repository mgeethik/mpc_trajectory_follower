#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

class TrajectoryTrackerNode : public rclcpp::Node
{
public:
    TrajectoryTrackerNode() : Node("trajectory_tracker_node")
    {
        // Parameters
        this->declare_parameter<double>("kp_angular", 2.5);
        this->get_parameter("kp_angular", kp_angular_);
        this->declare_parameter<double>("ki_angular", 0.5);
        this->get_parameter("ki_angular", ki_angular_);
        this->declare_parameter<double>("kd_angular", 0.5);
        this->get_parameter("kd_angular", kd_angular_);
        this->declare_parameter<double>("kp_cross_track", 0.5);
        this->get_parameter("kp_cross_track", kp_cross_track_);
        this->declare_parameter<double>("max_linear_vel", 0.22);
        this->get_parameter("max_linear_vel", max_linear_vel_);
        this->declare_parameter<double>("max_angular_vel", 2.84);
        this->get_parameter("max_angular_vel", max_angular_vel_);
        this->declare_parameter<double>("goal_tolerance", 0.1);
        this->get_parameter("goal_tolerance", goal_tolerance_);

        // Publishers and Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryTrackerNode::odom_callback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&TrajectoryTrackerNode::path_callback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        // Control loop timer
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz
            std::bind(&TrajectoryTrackerNode::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Trajectory Tracker Initialized.");
    }

private:
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        odom_received_ = true;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = *msg;
        path_received_ = true;
        path_index_ = 0; 
        integral_error_ = 0.0;
        last_error_ = 0.0;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "New trajectory received with %zu points.", path_.poses.size());
    }

    // Main control logic
    void control_loop()
    {
        if (!odom_received_ || !path_received_ || path_.poses.empty() || goal_reached_)
        {
            return;
        }

        if (path_index_ > path_.poses.size() * 0.8) 
        {
            double dist_to_goal = std::hypot(
                current_pose_.position.x - path_.poses.back().pose.position.x,
                current_pose_.position.y - path_.poses.back().pose.position.y);

            if (dist_to_goal < goal_tolerance_)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_reached_ = true;
                auto stop_cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
                stop_cmd->header.stamp = this->get_clock()->now();
                cmd_vel_pub_->publish(*stop_cmd);
                return;
            }
        }

        // Find the closest point on the path
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = path_index_; i < path_.poses.size(); ++i)
        {
            double dist_sq = std::pow(current_pose_.position.x - path_.poses[i].pose.position.x, 2) +
                             std::pow(current_pose_.position.y - path_.poses[i].pose.position.y, 2);
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        path_index_ = closest_idx;

        // Determine path angle at the closest point
        double path_angle = 0.0;
        // When at the end of the path, aim directly for the final point
        // instead of trying to match the last segment's angle. This creates a more stable "homing" behavior.
        if (path_index_ >= path_.poses.size() - 1)
        {
             path_angle = std::atan2(
                path_.poses.back().pose.position.y - current_pose_.position.y,
                path_.poses.back().pose.position.x - current_pose_.position.x);
        }
        else 
        {
            path_angle = std::atan2(
                path_.poses[path_index_ + 1].pose.position.y - path_.poses[path_index_].pose.position.y,
                path_.poses[path_index_ + 1].pose.position.x - path_.poses[path_index_].pose.position.x);
        }

        // Calculate cross-track error
        double dx = path_.poses[path_index_].pose.position.x - current_pose_.position.x;
        double dy = path_.poses[path_index_].pose.position.y - current_pose_.position.y;
        double cross_track_error = std::sin(path_angle - current_yaw_) * std::hypot(dx, dy);
        
        // Calculate heading error
        double heading_error = path_angle - current_yaw_;
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;

        // PID controller logic
        integral_error_ += heading_error;
        double derivative_error = heading_error - last_error_;
        last_error_ = heading_error;

        // Create and publish a TwistStamped message
        auto cmd_vel_stamped = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd_vel_stamped->header.stamp = this->get_clock()->now();
        
        cmd_vel_stamped->twist.angular.z = kp_angular_ * heading_error + 
                                           ki_angular_ * integral_error_ + 
                                           kd_angular_ * derivative_error +
                                           kp_cross_track_ * cross_track_error;

        // Limit angular velocity
        cmd_vel_stamped->twist.angular.z = std::clamp(cmd_vel_stamped->twist.angular.z, -max_angular_vel_, max_angular_vel_);
        
        // Reduce linear velocity during turns
        cmd_vel_stamped->twist.linear.x = max_linear_vel_ * (1.0 - 0.8 * std::abs(cmd_vel_stamped->twist.angular.z) / max_angular_vel_);

        cmd_vel_pub_->publish(*cmd_vel_stamped);
    }

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    geometry_msgs::msg::Pose current_pose_;
    double current_yaw_ = 0.0;
    bool odom_received_ = false;

    nav_msgs::msg::Path path_;
    bool path_received_ = false;
    size_t path_index_ = 0;

    double kp_angular_, ki_angular_, kd_angular_, kp_cross_track_;
    double max_linear_vel_, max_angular_vel_, goal_tolerance_;
    double integral_error_ = 0.0;
    double last_error_ = 0.0;
    bool goal_reached_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
    rclcpp::shutdown();
    return 0;
}

