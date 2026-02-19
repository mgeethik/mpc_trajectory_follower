# Theory: Algorithms and Approach

This document explains the core algorithms used in this navigation stack and the reasoning behind each choice.

---

## 1. Path Smoothing — Quintic Hermite Spline

A 5th-order polynomial spline that interpolates through every waypoint. Each segment between two consecutive waypoints is a degree-5 polynomial in x and y, parameterized by u ∈ [0, 1].

The polynomial coefficients are computed from boundary conditions at each endpoint — position, velocity, and acceleration — using the pre-computed 6×6 Hermite basis matrix M:

```
C = M · G      where G = [p₀, v₀, a₀, p₁, v₁, a₁]ᵀ
```

Endpoint accelerations are set to zero, which satisfies **C² continuity** at segment joins — position, velocity, and acceleration are all smooth everywhere. This is the minimum smoothness level for physically realistic robot motion, preventing torque spikes at waypoints.

Tangent vectors at each waypoint are computed adaptively based on the angle between incoming and outgoing path directions — sharper corners get smaller tangents to prevent the spline from swinging wide:

```
corner_factor = 1 - (1 - α) × (θ / π)
|tangent| = max_velocity × corner_factor
```

From the polynomial, heading θ, speed v, and angular velocity ω are derived **analytically** using the first and second derivatives — no finite-differencing noise:

```
θ = atan2(ẏ, ẋ)
ω = (ẋ·ÿ - ẏ·ẍ) / v²      (Frenet-Serret formula for planar curves)
```

This gives a complete trajectory `[x, y, θ, v, ω, t]` at every point.

---

## 2. Velocity Profile — Trapezoidal

The spline gives a geometric path parameterized by arc length. Timestamps are assigned using a trapezoidal velocity profile: accelerate at `max_acceleration` up to `max_velocity`, cruise, then decelerate symmetrically to a stop. For short paths that cannot reach cruise speed, the profile degenerates to a symmetric triangle.

This produces physically realistic speed commands that respect the robot's acceleration limits throughout the mission.

---

## 3. Obstacle Avoidance — Check-Correct-Smooth

A four-phase pipeline operating on the waypoint segments before spline generation:

1. **Check** — test each straight-line segment between waypoints for intersection with any obstacle's safety circle (physical radius + safety margin)
2. **Correct** — insert a perpendicular detour waypoint around any colliding segment, choosing the side (left or right) that minimizes total added path length
3. **Smooth** — run the Quintic Hermite spline through the corrected waypoints, naturally curving around the detours
4. **Verify** — re-check the final curved spline against all obstacles; if it still clips one, multiply the safety margin by 1.5 and retry up to 5 times

The verify step is necessary because a spline can bulge slightly inward toward an obstacle even when all waypoints are clear.

---

## 4. Trajectory Tracking — LPV-MPC

### Why MPC

An MPC controller optimizes over a **prediction horizon of N future steps**, seeing upcoming curves in the reference trajectory before the robot reaches them. It distributes corrections smoothly over the horizon and enforces hard velocity constraints across all N future steps simultaneously — something a reactive controller cannot do.

### The model

The unicycle (differential-drive) dynamics are:
```
ẋ = v·cos(θ),   ẏ = v·sin(θ),   θ̇ = ω
```

These are nonlinear. At each step k of the prediction horizon, the model is **linearized around the reference state** from the planned trajectory — giving a different linear model at each step (Linear Parameter-Varying):

```
z_{k+1} = A_k·z_k + B_k·u_k + d_k
```

The affine offset d_k captures the linearization error exactly by computing the difference between one true nonlinear step and the linear approximation at the reference point. Without it, the model would drift over the horizon.

### The optimization

All N predicted states are stacked into a condensed quadratic program. The cost minimized is:

```
J = Σ (z_k - z̄_k)ᵀ Q (z_k - z̄_k)  +  Σ uₖᵀ R uₖ
```

Where Q = diag(q_x, q_y, q_θ) penalizes position and heading error, and R = diag(r_v, r_ω) penalizes control effort. A larger terminal cost Q_f on the final step ensures the robot is well-aligned at the end of the horizon.

This is solved by **OSQP** — an interior-point QP solver that enforces `|v| ≤ v_max` and `|ω| ≤ ω_max` as hard inequalities across all steps simultaneously. OSQP **warm-starts** from the previous solution, which keeps solve time to 0.5–2ms and enables stable 50Hz operation.

### Predicted trajectory

After each solve, the optimal control sequence U* is forward-simulated through the condensed model to recover the full predicted state trajectory. This is published to `/visualization/mpc_horizon` and shown as the red path in RViz — always originating from the robot's current position, showing where the MPC calculates the robot will actually go given the optimal controls.

### Adaptive lookahead

The reference window fed to the MPC starts at the closest trajectory point and stretches forward adaptively with robot speed:

```
lookahead = clamp(lookahead_min + lookahead_gain × |v|, min, max)
```

Faster motion uses a longer lookahead so the horizon stays ahead of the robot on curves. Slower motion uses a tighter lookahead for more precise local tracking.

---

## 5. Code Architecture

The core math — spline generation and MPC — lives in **pure C++ header-only classes** (`spline_generator.hpp`, `mpc_controller.hpp`) with no ROS dependencies. The ROS nodes (`path_smoother_node.cpp`, `trajectory_tracker_node.cpp`) are thin wrappers that handle topic I/O and call into these classes.

This means the algorithms can be validated standalone without launching ROS or Gazebo:
```bash
./build/robot_navigation/test_spline
./build/robot_navigation/test_mpc
```

The full trajectory state `[x, y, θ, v, ω, t]` is passed between the smoother and tracker through the standard `nav_msgs/Path` message by encoding kinematic state into the orientation fields (`orientation.x = θ`, `orientation.y = v`, `orientation.z = ω`, `orientation.w = 1.0` as a sentinel). This avoids defining a custom message type while keeping the multi-node architecture clean.
