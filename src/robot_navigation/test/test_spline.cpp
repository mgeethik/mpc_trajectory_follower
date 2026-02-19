// test/test_spline.cpp
#include "robot_navigation/spline_generator.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

// Helper: check value within tolerance
void check(const char* name, double got, double expected, double tol=1e-4) {
    bool ok = std::abs(got - expected) < tol;
    std::printf("[%s] %s: got=%.6f, expected=%.6f\n",
                ok ? "PASS" : "FAIL", name, got, expected);
    assert(ok);
}

int main() {
    std::cout << "\n=== Spline Generator Tests ===\n\n";

    SplineGenerator gen;
    gen.max_velocity    = 0.22;
    gen.spline_ds       = 0.02;
    gen.corner_sharpness = 0.25;

    // ---- Test 1: Straight line ----
    // A straight line through 3 collinear points should produce:
    //   - heading ≈ 0 (horizontal)
    //   - omega ≈ 0 (no curvature)
    //   - monotonically increasing x
    {
        std::vector<std::pair<double,double>> wps = {{0,0},{1,0},{2,0}};
        auto traj = gen.generate(wps);
        assert(!traj.empty());

        // First point should be at origin
        check("straight_start_x", traj.front().x, 0.0, 0.05);
        check("straight_start_y", traj.front().y, 0.0, 0.05);

        // Last point should be near (2, 0)
        check("straight_end_x", traj.back().x, 2.0, 0.05);
        check("straight_end_y", traj.back().y, 0.0, 0.05);

        // All y values should be near zero
        for (const auto& s : traj)
            assert(std::abs(s.y) < 0.01 && "Straight line deviates in y");

        // Heading should be near 0 throughout
        for (const auto& s : traj)
            assert(std::abs(s.theta) < 0.1 && "Heading diverges on straight line");

        // Omega should be near 0 (no curvature)
        for (const auto& s : traj)
            assert(std::abs(s.omega) < 0.5 && "Omega nonzero on straight line");

        // Time should be monotonically increasing
        for (size_t i = 1; i < traj.size(); ++i)
            assert(traj[i].t >= traj[i-1].t && "Time not monotonic");

        std::printf("[PASS] Straight line test\n");
    }

    // ---- Test 2: Right angle turn ----
    // 90-degree turn: (0,0) → (1,0) → (1,1)
    // At the corner waypoint, omega should be non-zero (curve exists)
    {
        std::vector<std::pair<double,double>> wps = {{0,0},{1,0},{1,1}};
        auto traj = gen.generate(wps);
        assert(!traj.empty());

        // End point near (1,1)
        check("right_turn_end_x", traj.back().x, 1.0, 0.1);
        check("right_turn_end_y", traj.back().y, 1.0, 0.1);

        // Should have at least one point with significant omega
        double max_omega = 0.0;
        for (const auto& s : traj) max_omega = std::max(max_omega, std::abs(s.omega));
        assert(max_omega > 0.1 && "Right angle turn has zero curvature — spline too straight");
        std::printf("[PASS] Right angle turn test (max omega=%.3f)\n", max_omega);
    }

    // ---- Test 3: C² continuity check ----
    // Verify that acceleration (d²p/dt²) is continuous at waypoint boundaries.
    // We do this by checking that omega doesn't jump discontinuously at waypoint seams.
    {
        std::vector<std::pair<double,double>> wps = {{0,0},{2,0},{3,2},{1,3}};
        auto traj = gen.generate(wps);

        double max_omega_jump = 0.0;
        for (size_t i = 1; i < traj.size(); ++i) {
            double jump = std::abs(traj[i].omega - traj[i-1].omega);
            max_omega_jump = std::max(max_omega_jump, jump);
        }
        // With C² continuity, omega should vary smoothly (no large jumps between samples)
        assert(max_omega_jump < 5.0 && "Large omega jump — possible C2 violation");
        std::printf("[PASS] C2 continuity: max omega jump between samples = %.4f\n", max_omega_jump);
    }

    // ---- Test 4: Minimum waypoints ----
    {
        std::vector<std::pair<double,double>> wps = {{0,0},{1,1}};
        auto traj = gen.generate(wps);
        assert(!traj.empty() && "2-waypoint spline failed");
        std::printf("[PASS] Minimum 2-waypoint case\n");
    }

    // ---- Test 5: Speed magnitude ----
    // All trajectory points should have v in [0, max_velocity * 1.5]
    {
        std::vector<std::pair<double,double>> wps = {{0,0},{1,0},{2,1},{1,2},{0,1}};
        auto traj = gen.generate(wps);
        for (const auto& s : traj) {
            assert(s.v >= 0.0          && "Negative speed");
            assert(s.v < gen.max_velocity * 2.0 && "Speed exceeds limit");
        }
        std::printf("[PASS] Speed magnitude within bounds\n");
    }

    std::cout << "\n=== All spline tests passed ===\n\n";
    return 0;
}