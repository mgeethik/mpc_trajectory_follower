// include/robot_navigation/spline_generator.hpp
#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// ============================================================
// Full kinematic state at a single trajectory point
// ============================================================
struct TrajState {
    double x;      // position (m)
    double y;      // position (m)
    double theta;  // heading (rad)
    double v;      // linear speed (m/s)
    double omega;  // angular velocity (rad/s), derived analytically from curvature
    double t;      // timestamp (s)
};

// ============================================================
// SplineGenerator
//
// Generates a quintic Hermite spline through a set of 2D waypoints
// and returns a time-parameterized trajectory with full kinematic state.
//
// Guarantees:
//   - C² continuity (position, velocity, acceleration all continuous)
//   - Passes through every waypoint (interpolating, not approximating)
//   - Analytical heading and ω (no finite-differencing noise)
//   - Adaptive corner-sharpness: prevents overshoot on sharp turns
// ============================================================
class SplineGenerator {
public:
    // Hermite basis matrix M.
    // Solves: C = M * G  where G = [p0, v0, a0, p1, v1, a1]^T
    // Gives polynomial coefficients: p(u) = c0 + c1*u + ... + c5*u^5
    // Pre-computed offline to avoid runtime matrix inversion.
    static constexpr double M_[6][6] = {
        { 1.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        { 0.0,  1.0,  0.0,  0.0,  0.0,  0.0},
        { 0.0,  0.0,  0.5,  0.0,  0.0,  0.0},
        {-10.0, -6.0, -1.5, 10.0, -4.0,  0.5},
        { 15.0,  8.0,  1.5,-15.0,  7.0, -1.0},
        { -6.0, -3.0, -0.5,  6.0, -3.0,  0.5}
    };

    // Configuration
    double max_velocity;      // Target speed (m/s) — used for tangent scaling and timing
    double spline_ds;         // Arc-length spacing between output points (m)
    double corner_sharpness;  // [0.0 = very tight corners, 1.0 = very smooth/wide]

    SplineGenerator()
        : max_velocity(0.22), spline_ds(0.02), corner_sharpness(0.25)
    {}

    // -------------------------------------------------------
    // Main entry point.
    // waypoints: list of {x, y} pairs (at least 2 required)
    // Returns: time-ordered trajectory states
    // -------------------------------------------------------
    std::vector<TrajState> generate(const std::vector<std::pair<double,double>>& waypoints)
    {
        if (waypoints.size() < 2)
            throw std::invalid_argument("SplineGenerator: need at least 2 waypoints");

        int n = static_cast<int>(waypoints.size());

        // Step 1: Compute adaptive tangent vectors at each waypoint
        auto tangents = compute_tangents(waypoints);

        // Step 2: Build quintic Hermite segments between consecutive waypoints
        // Each segment stores 6 polynomial coefficients for x and y separately
        struct Segment {
            double cx[6], cy[6]; // polynomial coefficients
            double T;            // segment duration (s)
            double t0;           // global start time of this segment
        };

        std::vector<Segment> segments;
        double global_time = 0.0;

        for (int i = 0; i + 1 < n; ++i) {
            double p0x = waypoints[i].first,   p0y = waypoints[i].second;
            double p1x = waypoints[i+1].first, p1y = waypoints[i+1].second;
            double v0x = tangents[i].first,    v0y = tangents[i].second;
            double v1x = tangents[i+1].first,  v1y = tangents[i+1].second;
            // Accelerations at endpoints are set to zero.
            // Matching a_1 of segment i with a_0 of segment i+1 ensures C² continuity.
            const double a0x = 0.0, a0y = 0.0;
            const double a1x = 0.0, a1y = 0.0;

            // Segment duration from Euclidean distance and target speed
            double dist = std::hypot(p1x - p0x, p1y - p0y);
            double T = std::max(dist / max_velocity, 1e-6);

            // Normalize boundary conditions by T (Hermite formulation uses u ∈ [0,1])
            // dp/dt = (dp/du) / T → dp/du = (dp/dt) * T
            double Gx[6] = {p0x, v0x*T, a0x*T*T, p1x, v1x*T, a1x*T*T};
            double Gy[6] = {p0y, v0y*T, a0y*T*T, p1y, v1y*T, a1y*T*T};

            Segment seg;
            seg.T  = T;
            seg.t0 = global_time;

            // C = M * G  (matrix-vector multiply with precomputed M)
            for (int r = 0; r < 6; ++r) {
                seg.cx[r] = 0.0; seg.cy[r] = 0.0;
                for (int c = 0; c < 6; ++c) {
                    seg.cx[r] += M_[r][c] * Gx[c];
                    seg.cy[r] += M_[r][c] * Gy[c];
                }
            }

            segments.push_back(seg);
            global_time += T;
        }

        // Step 3: Discretize each segment, compute full kinematic state analytically
        std::vector<TrajState> trajectory;

        for (size_t s = 0; s < segments.size(); ++s) {
            const auto& seg = segments[s];
            double T = seg.T;

            // Number of samples based on estimated arc-length
            double seg_len = std::hypot(
                waypoints[s+1].first  - waypoints[s].first,
                waypoints[s+1].second - waypoints[s].second);
            int n_steps = std::max(2, static_cast<int>(seg_len / spline_ds));

            int start_step = (s == 0) ? 0 : 1; // skip duplicate at segment joins

            for (int step = start_step; step <= n_steps; ++step) {
                double u = static_cast<double>(step) / n_steps;
                double u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u;

                // Position: p(u)
                double px = seg.cx[0] + seg.cx[1]*u + seg.cx[2]*u2
                          + seg.cx[3]*u3 + seg.cx[4]*u4 + seg.cx[5]*u5;
                double py = seg.cy[0] + seg.cy[1]*u + seg.cy[2]*u2
                          + seg.cy[3]*u3 + seg.cy[4]*u4 + seg.cy[5]*u5;

                // Velocity: dp/du (divide by T to get dp/dt)
                double vx_du = seg.cx[1] + 2*seg.cx[2]*u + 3*seg.cx[3]*u2
                             + 4*seg.cx[4]*u3 + 5*seg.cx[5]*u4;
                double vy_du = seg.cy[1] + 2*seg.cy[2]*u + 3*seg.cy[3]*u2
                             + 4*seg.cy[4]*u3 + 5*seg.cy[5]*u4;
                double vx = vx_du / T;
                double vy = vy_du / T;

                // Acceleration: d²p/du² (divide by T² to get d²p/dt²)
                double ax_du2 = 2*seg.cx[2] + 6*seg.cx[3]*u + 12*seg.cx[4]*u2 + 20*seg.cx[5]*u3;
                double ay_du2 = 2*seg.cy[2] + 6*seg.cy[3]*u + 12*seg.cy[4]*u2 + 20*seg.cy[5]*u3;
                double ax = ax_du2 / (T*T);
                double ay = ay_du2 / (T*T);

                double v_mag = std::hypot(vx, vy);
                double theta = std::atan2(vy, vx);

                // Angular velocity (signed curvature * speed):
                //   ω = (vx*ay - vy*ax) / |v|²
                // Derived from the Frenet-Serret formula for planar curves.
                double omega = 0.0;
                if (v_mag > 0.01)
                    omega = (vx * ay - vy * ax) / (v_mag * v_mag);

                TrajState state;
                state.x     = px;
                state.y     = py;
                state.theta = theta;
                state.v     = v_mag;
                state.omega = omega;
                state.t     = seg.t0 + u * T;
                trajectory.push_back(state);
            }
        }

        return trajectory;
    }

private:
    // -------------------------------------------------------
    // Adaptive tangent computation.
    // Interior tangents are the normalized average of in/out directions,
    // scaled by a corner factor: sharper corners → smaller tangent magnitude
    // → prevents the spline from swinging wide around tight waypoints.
    // -------------------------------------------------------
    std::vector<std::pair<double,double>> compute_tangents(
        const std::vector<std::pair<double,double>>& wps)
    {
        int n = static_cast<int>(wps.size());
        std::vector<std::pair<double,double>> tangents(n, {0.0, 0.0});

        for (int i = 0; i < n; ++i) {
            double tx = 0.0, ty = 0.0;
            double corner_factor = 1.0;

            if (i == 0) {
                tx = wps[1].first  - wps[0].first;
                ty = wps[1].second - wps[0].second;
            } else if (i == n - 1) {
                tx = wps[n-1].first  - wps[n-2].first;
                ty = wps[n-1].second - wps[n-2].second;
            } else {
                // Incoming direction (normalized)
                double in_x = wps[i].first  - wps[i-1].first;
                double in_y = wps[i].second - wps[i-1].second;
                double norm_in = std::hypot(in_x, in_y);
                if (norm_in > 1e-9) { in_x /= norm_in; in_y /= norm_in; }

                // Outgoing direction (normalized)
                double out_x = wps[i+1].first  - wps[i].first;
                double out_y = wps[i+1].second - wps[i].second;
                double norm_out = std::hypot(out_x, out_y);
                if (norm_out > 1e-9) { out_x /= norm_out; out_y /= norm_out; }

                // Angle between segments (0 = straight, π = U-turn)
                double dot = std::clamp(in_x*out_x + in_y*out_y, -1.0, 1.0);
                double angle = std::acos(dot);

                // corner_factor: 1.0 at straight-through, corner_sharpness at U-turn
                corner_factor = 1.0 - (1.0 - corner_sharpness) * (angle / M_PI);

                // Average direction
                double avg_x = in_x + out_x;
                double avg_y = in_y + out_y;
                double avg_norm = std::hypot(avg_x, avg_y);

                if (avg_norm > 1e-9) {
                    tx = avg_x / avg_norm;
                    ty = avg_y / avg_norm;
                } else {
                    // 180° reversal: use perpendicular to incoming direction
                    tx = -in_y;
                    ty =  in_x;
                    corner_factor = corner_sharpness;
                }
            }

            // Normalize and scale by desired speed * corner_factor
            double norm = std::hypot(tx, ty);
            if (norm > 1e-9) {
                tangents[i] = {
                    (tx / norm) * max_velocity * corner_factor,
                    (ty / norm) * max_velocity * corner_factor
                };
            } else {
                tangents[i] = {max_velocity, 0.0};
            }
        }
        return tangents;
    }
};

// Required for constexpr static member in C++14/17
constexpr double SplineGenerator::M_[6][6];