#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_navigation/spline_generator.hpp"

#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

struct Point { double x, y; };

class PathSmootherNode : public rclcpp::Node {
public:
    PathSmootherNode() : Node("path_smoother_node")
    {
        // Parameters
        declare_parameter("velocity_profile",     std::string("trapezoidal"));
        declare_parameter("max_velocity",         0.22);
        declare_parameter("max_acceleration",     0.10);
        declare_parameter("time_sample_interval", 0.1);
        declare_parameter("spline_ds",            0.02);
        declare_parameter("corner_sharpness",     0.25);
        declare_parameter("obstacle_radius",      0.35);
        declare_parameter("safety_margin",        0.15);
        declare_parameter("obstacle_centers",     std::vector<std::string>{});

        get_parameter("velocity_profile",     velocity_profile_);
        get_parameter("max_velocity",         max_vel_);
        get_parameter("max_acceleration",     max_acc_);
        get_parameter("time_sample_interval", dt_);
        get_parameter("spline_ds",            spline_ds_);
        get_parameter("corner_sharpness",     corner_sharpness_);
        get_parameter("obstacle_radius",      obs_r_);
        get_parameter("safety_margin",        safety_margin_);

        
        auto obstacle_strings = get_parameter("obstacle_centers").as_string_array();
        for (const auto& s : obstacle_strings)
            obstacles_.push_back(parse_point(s));

        // Configure SplineGenerator
        spline_.max_velocity    = max_vel_;
        spline_.spline_ds       = spline_ds_;
        spline_.corner_sharpness = corner_sharpness_;

        // Pub / Sub
        path_sub_  = create_subscription<nav_msgs::msg::Path>(
            "/goal_waypoints", 10,
            std::bind(&PathSmootherNode::on_path, this, std::placeholders::_1));
        path_pub_  = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        vel_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/visualization/velocity_profile", 10);
        omega_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/visualization/omega_profile", 10);

        RCLCPP_INFO(get_logger(), "Path Smoother ready (Quintic Hermite, %s profile)",
                    velocity_profile_.c_str());
    }

private:
    // -------- Main callback --------
    void on_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(get_logger(), "Need at least 2 waypoints."); return;
        }

        // Extract raw waypoints
        std::vector<std::pair<double,double>> raw_wps;
        for (const auto& ps : msg->poses)
            raw_wps.push_back({ps.pose.position.x, ps.pose.position.y});

        // Obstacle avoidance: Check-Correct-Smooth with iterative verification
        double margin = safety_margin_;
        std::vector<std::pair<double,double>> corrected_wps;
        bool ok = false;
        for (int iter = 0; iter < 5; ++iter) {
            corrected_wps = correct_for_obstacles(raw_wps, margin);
            if (corrected_wps.empty()) {
                RCLCPP_ERROR(get_logger(), "Waypoint inside obstacle — cannot plan."); return;
            }
            if (is_collision_free(corrected_wps)) { ok = true; break; }
            margin *= 1.5;
            RCLCPP_WARN(get_logger(), "Spline collides, retrying with margin=%.2f", margin);
        }
        if (!ok) { RCLCPP_ERROR(get_logger(), "No collision-free path found."); return; }

        // Generate quintic Hermite trajectory (full kinematic state)
        spline_.max_velocity     = max_vel_;
        spline_.spline_ds        = spline_ds_;
        spline_.corner_sharpness = corner_sharpness_;
        auto geom_traj = spline_.generate(corrected_wps);

        // Apply velocity profile (trapezoidal or constant)
        auto timed_traj = apply_velocity_profile(geom_traj);

        // Publish
        publish_trajectory(timed_traj);
        publish_debug_profiles(timed_traj);

        RCLCPP_INFO(get_logger(), "Published %zu trajectory points.", timed_traj.size());
    }

    // -------- Velocity profile --------
    std::vector<TrajState> apply_velocity_profile(const std::vector<TrajState>& geom)
    {
        if (geom.empty()) return {};

        // Build cumulative arc-length
        std::vector<double> arc(geom.size(), 0.0);
        for (size_t i = 1; i < geom.size(); ++i)
            arc[i] = arc[i-1] + std::hypot(geom[i].x - geom[i-1].x,
                                            geom[i].y - geom[i-1].y);
        double L = arc.back();

        // Trapezoidal profile parameters
        double t_ramp = max_vel_ / max_acc_;
        double d_ramp = 0.5 * max_acc_ * t_ramp * t_ramp;
        if (L < 2.0 * d_ramp) {        // Path too short — triangle profile
            t_ramp = std::sqrt(L / max_acc_);
            d_ramp = 0.5 * L;
        }
        double t_cruise_end = t_ramp + (L - 2.0 * d_ramp) / max_vel_;
        double t_total      = t_cruise_end + t_ramp;

        std::vector<TrajState> timed;
        double t = 0.0;

        auto interp = [&](double dist) -> TrajState {
            // Find segment in arc-length parameterized geom trajectory
            size_t idx = 0;
            while (idx + 1 < arc.size() && arc[idx+1] < dist) ++idx;
            if (idx + 1 >= geom.size()) return geom.back();
            double frac = (arc[idx+1] - arc[idx] > 1e-9)
                ? (dist - arc[idx]) / (arc[idx+1] - arc[idx]) : 0.0;
            TrajState s;
            s.x     = geom[idx].x     + frac * (geom[idx+1].x - geom[idx].x);
            s.y     = geom[idx].y     + frac * (geom[idx+1].y - geom[idx].y);
            s.theta = geom[idx].theta;
            s.omega = geom[idx].omega;
            s.v     = 0.0;  // will be set by profile
            s.t     = 0.0;
            return s;
        };

        while (true) {
            double dist = 0.0, vel = 0.0;
            if (velocity_profile_ == "constant") {
                vel  = max_vel_;
                dist = max_vel_ * t;
            } else {
                if (t <= t_ramp) {
                    vel  = max_acc_ * t;
                    dist = 0.5 * max_acc_ * t * t;
                } else if (t <= t_cruise_end) {
                    vel  = max_vel_;
                    dist = d_ramp + max_vel_ * (t - t_ramp);
                } else if (t <= t_total) {
                    double td = t - t_cruise_end;
                    vel  = max_vel_ - max_acc_ * td;
                    dist = d_ramp + max_vel_*(t_cruise_end - t_ramp)
                           + max_vel_*td - 0.5*max_acc_*td*td;
                } else {
                    dist = L;
                }
            }
            if (dist >= L) break;

            TrajState s = interp(dist);
            s.v = vel;
            s.t = t;
            timed.push_back(s);
            t += dt_;
        }
        TrajState final = geom.back();
        final.v = 0.0; final.t = t;
        timed.push_back(final);
        return timed;
    }

    // -------- Publish trajectory (encode kinematic state into orientation fields) --------
    // Convention: orientation.{x,y,z} = {θ, v, ω}, orientation.w = 1.0 (sentinel)
    // This avoids defining a custom ROS message while keeping multi-node decoupling.
    void publish_trajectory(const std::vector<TrajState>& traj)
    {
        nav_msgs::msg::Path msg;
        msg.header.stamp    = get_clock()->now();
        msg.header.frame_id = "odom";

        for (const auto& s : traj) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.header.stamp = rclcpp::Time(msg.header.stamp)
                              + rclcpp::Duration::from_seconds(s.t);
            ps.pose.position.x       = s.x;
            ps.pose.position.y       = s.y;
            ps.pose.position.z       = 0.0;
            ps.pose.orientation.x    = s.theta;
            ps.pose.orientation.y    = s.v;
            ps.pose.orientation.z    = s.omega;
            ps.pose.orientation.w    = 1.0;   // sentinel: marks encoded state
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

    // -------- Publish debug profiles --------
    void publish_debug_profiles(const std::vector<TrajState>& traj)
    {
        std_msgs::msg::Float64MultiArray vel_msg, omega_msg;
        for (const auto& s : traj) {
            vel_msg.data.push_back(s.t);
            vel_msg.data.push_back(s.v);
            omega_msg.data.push_back(s.t);
            omega_msg.data.push_back(s.omega);
        }
        vel_pub_->publish(vel_msg);
        omega_pub_->publish(omega_msg);
    }

    // -------- Obstacle avoidance (unchanged logic from original) --------
    bool seg_circle_intersect(Point a, Point b, Point c, double r) {
        double dx=b.x-a.x, dy=b.y-a.y, fx=a.x-c.x, fy=a.y-c.y;
        double A=dx*dx+dy*dy, B=2*(fx*dx+fy*dy);
        double disc = B*B - 4*A*(fx*fx+fy*fy-r*r);
        if (disc < 0) return false;
        disc = std::sqrt(disc);
        double t1=(-B-disc)/(2*A), t2=(-B+disc)/(2*A);
        return (t1>=0&&t1<=1)||(t2>=0&&t2<=1);
    }

    std::vector<std::pair<double,double>> correct_for_obstacles(
        const std::vector<std::pair<double,double>>& wps, double margin)
    {
        double eff_r = obs_r_ + margin;
        std::vector<std::pair<double,double>> out;
        out.push_back(wps[0]);
        for (size_t i = 0; i+1 < wps.size(); ++i) {
            Point p1{out.back().first, out.back().second};
            Point p2{wps[i+1].first, wps[i+1].second};
            for (const auto& obs : obstacles_) {
                if (seg_circle_intersect(p1, p2, obs, eff_r)) {
                    double dx=p2.x-p1.x, dy=p2.y-p1.y, mag=std::hypot(dx,dy);
                    Point perp{-dy/mag, dx/mag};
                    Point d1{obs.x+perp.x*eff_r, obs.y+perp.y*eff_r};
                    Point d2{obs.x-perp.x*eff_r, obs.y-perp.y*eff_r};
                    double c1=std::hypot(p1.x-d1.x,p1.y-d1.y)+std::hypot(p2.x-d1.x,p2.y-d1.y);
                    double c2=std::hypot(p1.x-d2.x,p1.y-d2.y)+std::hypot(p2.x-d2.x,p2.y-d2.y);
                    Point best = (c1<c2) ? d1 : d2;
                    out.push_back({best.x, best.y});
                    break;
                }
            }
            out.push_back({p2.x, p2.y});
        }
        return out;
    }

    bool is_collision_free(const std::vector<std::pair<double,double>>& path) {
        for (size_t i = 0; i+1 < path.size(); ++i)
            for (const auto& obs : obstacles_) {
                Point a{path[i].first,path[i].second}, b{path[i+1].first,path[i+1].second};
                if (seg_circle_intersect(a, b, obs, obs_r_)) return false;
            }
        return true;
    }

    Point parse_point(const std::string& s) {
        std::string c = s;
        for (char& ch : c) if (ch=='['||ch==']'||ch==',') ch=' ';
        std::stringstream ss(c); Point p{0,0}; ss>>p.x>>p.y; return p;
    }

    // Members
    SplineGenerator spline_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_, omega_pub_;

    std::string velocity_profile_;
    double max_vel_, max_acc_, dt_, spline_ds_, corner_sharpness_;
    double obs_r_, safety_margin_;
    std::vector<Point> obstacles_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSmootherNode>());
    rclcpp::shutdown();
}