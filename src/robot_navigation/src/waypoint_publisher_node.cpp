#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <cctype>

// Simple struct for 2D points
struct Point { double x, y; };

class WaypointPublisherNode : public rclcpp::Node
{
public:
    WaypointPublisherNode() : Node("waypoint_publisher_node")
    {
        this->declare_parameter<std::string>("waypoint_file_path", "");
        this->get_parameter("waypoint_file_path", waypoint_file_path_);
        this->declare_parameter<double>("obstacle_radius", 0.5);
        this->get_parameter("obstacle_radius", obstacle_radius_);
        
        // **FIX**: Declare and parse obstacle_centers as a string array
        this->declare_parameter<std::vector<std::string>>("obstacle_centers", std::vector<std::string>{});
        auto obstacle_strings = this->get_parameter("obstacle_centers").as_string_array();
        for (const auto& s : obstacle_strings) {
            obstacles_.push_back(parse_point_string(s));
        }

        rclcpp::QoS latching_qos(1);
        latching_qos.transient_local();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/goal_waypoints", 10);
        waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/waypoints", latching_qos);
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/obstacles", latching_qos);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointPublisherNode::publish_all, this));
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

    void publish_all()
    {
        timer_->cancel();
        publish_waypoints();
        publish_waypoint_markers();
        publish_obstacle_markers();
        RCLCPP_INFO(this->get_logger(), "Published visualization markers.");
    }

    void publish_waypoints() {
        RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", waypoint_file_path_.c_str());
        YAML::Node config = YAML::LoadFile(waypoint_file_path_);
        const YAML::Node& waypoints_yaml = config["waypoints"];

        auto path_msg = std::make_unique<nav_msgs::msg::Path>();
        path_msg->header.stamp = this->get_clock()->now();
        path_msg->header.frame_id = "odom";

        if (!waypoints_yaml) {
            RCLCPP_ERROR(this->get_logger(), "'waypoints' key not found in %s", waypoint_file_path_.c_str());
            return;
        }

        for (const auto& waypoint_node : waypoints_yaml) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg->header;
            pose.pose.position.x = waypoint_node[0].as<double>();
            pose.pose.position.y = waypoint_node[1].as<double>();
            pose.pose.orientation.w = 1.0;
            path_msg->poses.push_back(pose);
        }
        
        path_pub_->publish(*path_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu waypoints.", path_msg->poses.size());
    }

    void publish_waypoint_markers() {
        YAML::Node config = YAML::LoadFile(waypoint_file_path_);
        const YAML::Node& waypoints_yaml = config["waypoints"];
        if (!waypoints_yaml) return;
        
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& waypoint_node : waypoints_yaml) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = waypoint_node[0].as<double>();
            marker.pose.position.y = waypoint_node[1].as<double>();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        waypoint_marker_pub_->publish(marker_array);
    }

    void publish_obstacle_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& obs : obstacles_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = obs.x;
            marker.pose.position.y = obs.y;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = obstacle_radius_ * 2.0;
            marker.scale.y = obstacle_radius_ * 2.0;
            marker.scale.z = 0.1;
            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        obstacle_marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string waypoint_file_path_;
    double obstacle_radius_;
    std::vector<Point> obstacles_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

