// test/test_mpc.cpp
#include "robot_navigation/mpc_controller.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    std::cout << "\n=== LPV-MPC Controller Tests ===\n\n";

    LPVMPCController mpc(10, 0.05);  // Short horizon for test speed
    mpc.v_max = 0.5; mpc.v_min = -0.1; mpc.w_max = 2.0;
    mpc.q_x = 100; mpc.q_y = 100; mpc.q_theta = 50;
    mpc.r_v = 20;  mpc.r_w = 20;
    mpc.terminal_scale = 3.0;

    // ---- Test 1: Forward tracking ----
    // Robot at (0,0,0), ref trajectory is straight line ahead along x-axis.
    // Expected: positive v, near-zero omega.
    {
        Eigen::Vector3d x0(0.0, 0.0, 0.0);
        std::vector<RefState> refs;
        for (int k = 0; k <= 10; ++k) {
            RefState r;
            r.x = k * 0.5 * 0.05;  // 0.5 m/s * dt * k
            r.y = 0.0;
            r.theta = 0.0;
            r.v = 0.5;
            r.omega = 0.0;
            refs.push_back(r);
        }
        auto sol = mpc.solve(x0, refs);
        Eigen::Vector2d u = sol.u;
        std::printf("Forward tracking: v=%.4f, omega=%.4f\n", u(0), u(1));
        assert(u(0) > 0.0   && "MPC should command positive speed for forward tracking");
        assert(u(0) <= mpc.v_max && "Speed exceeds v_max constraint");
        assert(std::abs(u(1)) <= mpc.w_max && "omega exceeds w_max constraint");
        assert(std::abs(u(1)) < 0.5 && "Omega too large for straight-line tracking");
        std::printf("[PASS] Forward tracking\n");
    }

    // ---- Test 2: Left-turn correction ----
    // Robot at (0,0,0) facing along x, ref is at y=+1 (to the left).
    // Expected: MPC should command positive omega to turn left.
    {
        mpc.reset();
        Eigen::Vector3d x0(0.0, 0.0, 0.0);
        std::vector<RefState> refs;
        for (int k = 0; k <= 10; ++k) {
            RefState r;
            r.x = 0.0; r.y = 1.0 + k * 0.01;
            r.theta = M_PI / 2.0;
            r.v = 0.2; r.omega = 0.0;
            refs.push_back(r);
        }
        auto sol = mpc.solve(x0, refs);
        Eigen::Vector2d u = sol.u;
        std::printf("Left-turn correction: v=%.4f, omega=%.4f\n", u(0), u(1));
        assert(u(1) > 0.0 && "MPC should command positive omega to turn left");
        assert(u(1) <= mpc.w_max && "omega exceeds w_max constraint");
        std::printf("[PASS] Left-turn correction\n");
    }

    // ---- Test 3: Already at reference ----
    // Robot is exactly at the reference trajectory. Commands should be near-zero.
    {
        mpc.reset();
        Eigen::Vector3d x0(1.0, 1.0, 0.5);
        std::vector<RefState> refs;
        for (int k = 0; k <= 10; ++k) {
            RefState r;
            r.x = 1.0; r.y = 1.0; r.theta = 0.5;
            r.v = 0.0; r.omega = 0.0;
            refs.push_back(r);
        }
        auto sol = mpc.solve(x0, refs);
        Eigen::Vector2d u = sol.u;
        std::printf("At reference: v=%.6f, omega=%.6f\n", u(0), u(1));
        assert(std::abs(u(0)) < 0.05 && "Speed should be near-zero when at reference");
        assert(std::abs(u(1)) < 0.2  && "omega should be near-zero when at reference");
        std::printf("[PASS] At reference (near-zero commands)\n");
    }

    // ---- Test 4: Constraint enforcement ----
    // Reference far away — even if unconstrained optimum exceeds v_max,
    // OSQP must clamp within bounds.
    {
        mpc.reset();
        Eigen::Vector3d x0(0.0, 0.0, 0.0);
        std::vector<RefState> refs;
        for (int k = 0; k <= 10; ++k) {
            RefState r;
            r.x = 100.0; r.y = 0.0; r.theta = 0.0;  // Very far away
            r.v = 5.0; r.omega = 0.0;  // Ref speed > v_max
            refs.push_back(r);
        }
        auto sol = mpc.solve(x0, refs);
        Eigen::Vector2d u = sol.u;
        std::printf("Constraint enforcement: v=%.4f (max=%.4f)\n", u(0), mpc.v_max);
        assert(u(0) <= mpc.v_max + 1e-4 && "v violates v_max constraint");
        assert(u(0) >= mpc.v_min - 1e-4 && "v violates v_min constraint");
        std::printf("[PASS] Constraint enforcement\n");
    }

    // ---- Test 5: Warm-start consistency ----
    // Solving the same problem twice should give the same result.
    {
        mpc.reset();
        Eigen::Vector3d x0(0.0, 0.0, 0.0);
        std::vector<RefState> refs;
        for (int k = 0; k <= 10; ++k) {
            RefState r; r.x=k*0.01; r.y=0; r.theta=0; r.v=0.2; r.omega=0;
            refs.push_back(r);
        }
        auto sol1 = mpc.solve(x0, refs);
        auto sol2 = mpc.solve(x0, refs);
        Eigen::Vector2d u1 = sol1.u;
        Eigen::Vector2d u2 = sol2.u;   // Same problem, warm-started
        double diff = (u1 - u2).norm();
        std::printf("Warm-start diff: %.8f\n", diff);
        assert(diff < 1e-4 && "Warm-start changes solution for identical problem");
        std::printf("[PASS] Warm-start consistency\n");
    }

    std::cout << "\n=== All MPC tests passed ===\n\n";
    return 0;
}