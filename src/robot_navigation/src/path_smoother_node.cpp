#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" 
#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>
#include <sstream>

// ... (Point struct remains the same) ...
struct Point { double x, y; };

class PathSmootherNode : public rclcpp::Node
{
public:
    PathSmootherNode() : Node("path_smoother_node")
    {
        // --- Parameters ---
        this->declare_parameter<std::string>("velocity_profile", "trapezoidal");
        this->get_parameter("velocity_profile", velocity_profile_);
        this->declare_parameter<double>("max_velocity", 0.5);
        this->get_parameter("max_velocity", max_velocity_);
        this->declare_parameter<double>("max_acceleration", 0.25);
        this->get_parameter("max_acceleration", max_acceleration_);
        this->declare_parameter<double>("time_sample_interval", 0.1);
        this->get_parameter("time_sample_interval", time_sample_interval_);
        this->declare_parameter<double>("spline_resolution", 0.05);
        this->get_parameter("spline_resolution", spline_resolution_);
        this->declare_parameter<double>("obstacle_radius", 0.35);
        this->get_parameter("obstacle_radius", obstacle_radius_);
        this->declare_parameter<double>("safety_margin", 0.15);
        this->get_parameter("safety_margin", safety_margin_);
        this->declare_parameter<double>("verification_resolution", 0.01);
        this->get_parameter("verification_resolution", verification_resolution_);

        // Declare and parse obstacle_centers as a string array
        this->declare_parameter<std::vector<std::string>>("obstacle_centers", {});
        auto obstacle_strings = this->get_parameter("obstacle_centers").as_string_array();
        for (const auto& s : obstacle_strings) {
            obstacles_.push_back(parse_point_string(s));
        }
        
        // --- Publishers & Subscribers ---
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/goal_waypoints", 10, std::bind(&PathSmootherNode::path_callback, this, std::placeholders::_1));
        smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        velocity_profile_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/visualization/velocity_profile", 10);

        RCLCPP_INFO(this->get_logger(), "Path Smoother Initialized.");
    }

private:
    // Helper to parse strings like "[x, y]"
    Point parse_point_string(const std::string& s) {
        Point p = {0.0, 0.0};
        std::string cleaned = s;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
        std::stringstream ss(cleaned);
        ss >> p.x >> p.y;
        return p;
    }

    // --- Main Callback ---
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received path has fewer than 2 points. Cannot process.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints, starting smoothing and obstacle avoidance...", msg->poses.size());

        std::vector<Point> initial_waypoints;
        for (const auto& pose_stamped : msg->poses) {
            initial_waypoints.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
        }
        
        auto geometric_path = generate_geometric_path(initial_waypoints);
        if (geometric_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate a collision-free geometric path.");
            return;
        }

        auto final_trajectory = generate_time_parametrized_trajectory(geometric_path);

        smooth_path_pub_->publish(final_trajectory);
        RCLCPP_INFO(this->get_logger(), "Successfully published smooth trajectory with %zu points.", final_trajectory.poses.size());
    }


    std::vector<Point> generate_geometric_path(const std::vector<Point>& waypoints)
    {
        // Phase 1 & 2: Check and Correct
        auto corrected_waypoints = correct_path_for_obstacles(waypoints);
        if (corrected_waypoints.empty()){
            RCLCPP_ERROR(this->get_logger(), "Path correction failed. A waypoint might be inside an obstacle.");
            return {};
        }

        // Phase 3: Smooth (with verification loop)
        double current_safety_margin = safety_margin_;
        for (int i = 0; i < 5; ++i) { // Limit iterations to prevent infinite loops
            auto candidate_spline = generate_candidate_spline(corrected_waypoints);
            
            if (is_path_collision_free(candidate_spline)) {
                RCLCPP_INFO(this->get_logger(), "Path is collision-free after %d iterations.", i + 1);
                return candidate_spline;
            }
            
            RCLCPP_WARN(this->get_logger(), "Path verification failed. Increasing safety margin and retrying...");
            current_safety_margin *= 1.5;
            corrected_waypoints = correct_path_for_obstacles(waypoints, current_safety_margin);
        }

        RCLCPP_ERROR(this->get_logger(), "Could not find a collision-free path after multiple attempts.");
        return {};
    }
    
    std::vector<Point> correct_path_for_obstacles(const std::vector<Point>& waypoints, double margin_override = -1.0)
    {
        // Check if any waypoint is inside an obstacle first
        for(const auto& wp : waypoints){
            for(const auto& obs : obstacles_){
                if(std::hypot(wp.x - obs.x, wp.y - obs.y) < obstacle_radius_){
                    RCLCPP_ERROR(this->get_logger(), "A waypoint is inside an obstacle! Path is impossible.");
                    return {};
                }
            }
        }

        double effective_radius = obstacle_radius_ + (margin_override > 0 ? margin_override : safety_margin_);
        std::vector<Point> corrected_path;
        corrected_path.push_back(waypoints[0]);

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            Point p1 = corrected_path.back();
            Point p2 = waypoints[i+1];
            
            int conflicting_obstacle_idx = -1;
            for (size_t j = 0; j < obstacles_.size(); ++j) {
                if (line_segment_circle_intersection(p1, p2, obstacles_[j], effective_radius)) {
                    conflicting_obstacle_idx = j;
                    break;
                }
            }

            if (conflicting_obstacle_idx != -1) {
                auto detour_points = find_detour_points(p1, p2, obstacles_[conflicting_obstacle_idx], effective_radius);
                if (detour_points.empty()) return {};
                corrected_path.insert(corrected_path.end(), detour_points.begin(), detour_points.end());
            }
            corrected_path.push_back(p2);
        }
        return corrected_path;
    }

    std::vector<Point> find_detour_points(Point p1, Point p2, Point obs_center, double obs_radius)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double mag = std::hypot(dx, dy);
        Point perp_vec = {-dy/mag, dx/mag};

        Point detour1 = {obs_center.x + perp_vec.x * obs_radius, obs_center.y + perp_vec.y * obs_radius};
        Point detour2 = {obs_center.x - perp_vec.x * obs_radius, obs_center.y - perp_vec.y * obs_radius};

        double dist1 = std::hypot(p1.x - detour1.x, p1.y - detour1.y) + std::hypot(p2.x - detour1.x, p2.y - detour1.y);
        double dist2 = std::hypot(p1.x - detour2.x, p1.y - detour2.y) + std::hypot(p2.x - detour2.x, p2.y - detour2.y);
        
        return (dist1 < dist2) ? std::vector<Point>{detour1} : std::vector<Point>{detour2};
    }

    bool line_segment_circle_intersection(Point p1, Point p2, Point circle_center, double radius)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double fx = p1.x - circle_center.x;
        double fy = p1.y - circle_center.y;
        double a = dx*dx + dy*dy;
        double b = 2*(fx*dx + fy*dy);
        double c = fx*fx + fy*fy - radius*radius;
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) return false;
        
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2*a);
        double t2 = (-b + discriminant) / (2*a);

        return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    }
    
    bool is_path_collision_free(const std::vector<Point>& path) {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            for(const auto& obs : obstacles_) {
                if (line_segment_circle_intersection(path[i], path[i+1], obs, obstacle_radius_)) {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<Point> generate_candidate_spline(const std::vector<Point>& points)
    {
        std::vector<Point> spline_points;
        if (points.size() < 2) return spline_points;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            Point p0 = (i == 0) ? points[i] : points[i - 1];
            Point p1 = points[i];
            Point p2 = points[i + 1];
            Point p3 = (i + 2 < points.size()) ? points[i + 2] : p2;

            for (double t = 0.0; t < 1.0; t += (spline_resolution_ / std::hypot(p2.x-p1.x, p2.y-p1.y))) {
                spline_points.push_back(catmull_rom_interpolate(t, p0, p1, p2, p3));
            }
        }
        spline_points.push_back(points.back());
        return spline_points;
    }

    Point catmull_rom_interpolate(double t, Point p0, Point p1, Point p2, Point p3) {
        double t2 = t * t;
        double t3 = t2 * t;
        double c1 = -0.5*t3 + t2 - 0.5*t;
        double c2 =  1.5*t3 - 2.5*t2 + 1.0;
        double c3 = -1.5*t3 + 2.0*t2 + 0.5*t;
        double c4 =  0.5*t3 - 0.5*t2;
        return {c1*p0.x + c2*p1.x + c3*p2.x + c4*p3.x,
                c1*p0.y + c2*p1.y + c3*p2.y + c4*p3.y};
    }

    // --- generate_time_parametrized_trajectory publishes velocity profile ---
    nav_msgs::msg::Path generate_time_parametrized_trajectory(const std::vector<Point>& geometric_path)
    {
        nav_msgs::msg::Path trajectory;
        trajectory.header.stamp = this->get_clock()->now();
        trajectory.header.frame_id = "odom";
        if (geometric_path.empty()) return trajectory;

        // For velocity profile visualization
        std_msgs::msg::Float64MultiArray velocity_profile_msg;
        velocity_profile_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        velocity_profile_msg.layout.dim[0].label = "time_velocity_pairs";
        velocity_profile_msg.layout.dim[0].stride = 2;

        double total_length = 0.0;
        for (size_t i = 0; i < geometric_path.size() - 1; ++i) {
            total_length += std::hypot(geometric_path[i+1].x - geometric_path[i].x, geometric_path[i+1].y - geometric_path[i].y);
        }

        double current_time = 0.0;
        
        while (true) {
            double current_dist = 0.0;
            double current_vel = 0.0;

            if (velocity_profile_ == "constant") {
                current_vel = max_velocity_;
                current_dist = max_velocity_ * current_time;
            } else { // trapezoidal
                double t_accel = max_velocity_ / max_acceleration_;
                double d_accel = 0.5 * max_acceleration_ * t_accel * t_accel;

                if (total_length < 2 * d_accel) { 
                    t_accel = std::sqrt(total_length / max_acceleration_);
                    d_accel = 0.5 * total_length;
                }
                
                double t_total = 2 * t_accel + (total_length - 2 * d_accel) / max_velocity_;
                double t_coast_end = t_total - t_accel;

                if (current_time <= t_accel) {
                    current_vel = max_acceleration_ * current_time;
                    current_dist = 0.5 * max_acceleration_ * current_time * current_time;
                } else if (current_time <= t_coast_end) {
                    current_vel = max_velocity_;
                    current_dist = d_accel + max_velocity_ * (current_time - t_accel);
                } else if (current_time <= t_total) {
                    double t_decel = current_time - t_coast_end;
                    current_vel = max_velocity_ - max_acceleration_ * t_decel;
                    current_dist = d_accel + max_velocity_ * (t_coast_end - t_accel) + (max_velocity_ * t_decel - 0.5 * max_acceleration_ * t_decel * t_decel);
                } else {
                    current_vel = 0.0;
                    current_dist = total_length;
                }
            }

            if (current_dist >= total_length) break;

            Point p = get_point_on_path(geometric_path, current_dist);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = trajectory.header;
            pose.header.stamp = rclcpp::Time(trajectory.header.stamp) + rclcpp::Duration::from_seconds(current_time);
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.orientation.w = 1.0;
            trajectory.poses.push_back(pose);

            // Add data to velocity profile message
            velocity_profile_msg.data.push_back(current_time);
            velocity_profile_msg.data.push_back(current_vel);

            current_time += time_sample_interval_;
        }
        
        geometry_msgs::msg::PoseStamped final_pose;
        final_pose.header = trajectory.header;
        final_pose.header.stamp = rclcpp::Time(trajectory.header.stamp) + rclcpp::Duration::from_seconds(current_time);
        final_pose.pose.position.x = geometric_path.back().x;
        final_pose.pose.position.y = geometric_path.back().y;
        final_pose.pose.orientation.w = 1.0;
        trajectory.poses.push_back(final_pose);
        
        velocity_profile_msg.data.push_back(current_time);
        velocity_profile_msg.data.push_back(0.0);
        velocity_profile_pub_->publish(velocity_profile_msg);

        return trajectory;
    }

    Point get_point_on_path(const std::vector<Point>& path, double distance)
    {
        double current_dist = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            double segment_len = std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
            if (current_dist + segment_len >= distance) {
                double ratio = (distance - current_dist) / segment_len;
                return {path[i].x + ratio * (path[i+1].x - path[i].x),
                        path[i].y + ratio * (path[i+1].y - path[i].y)};
            }
            current_dist += segment_len;
        }
        return path.back();
    }
    
    // --- Member Variables ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_profile_pub_;
    
    std::string velocity_profile_;
    double max_velocity_, max_acceleration_, time_sample_interval_, spline_resolution_;
    double obstacle_radius_, safety_margin_, verification_resolution_;
    std::vector<Point> obstacles_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSmootherNode>());
    rclcpp::shutdown();
    return 0;
}

