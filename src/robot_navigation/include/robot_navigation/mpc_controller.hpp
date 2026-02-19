// include/robot_navigation/mpc_controller.hpp
#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

// Reference state at one step of the MPC horizon
struct RefState {
    double x, y, theta;  // position and heading
    double v, omega;     // reference speed and angular velocity (feedforward)
};

// ============================================================
// LPVMPCController
//
// Linear Parameter-Varying Model Predictive Controller for a
// differential-drive (unicycle) robot.
//
// State:  z = [px, py, θ]      (3-dimensional)
// Input:  u = [v, ω]           (2-dimensional)
// Model:  Discrete-time unicycle, linearized at each reference point
//         along the prediction horizon (LPV = different A_k at each step)
//
// QP is solved in condensed form using OSQP with warm-starting.
// ============================================================
class LPVMPCController {
public:
    int    N;          // Prediction horizon (steps)
    double dt;         // Control time step (s)

    // Box constraints
    double v_max, v_min, w_max;

    // Cost weights
    double q_x, q_y, q_theta;      // State tracking weights
    double r_v, r_w;               // Control effort weights
    double terminal_scale;          // Terminal cost multiplier (Q_f = terminal_scale * Q)

    LPVMPCController(int horizon = 20, double timestep = 0.02)
        : N(horizon), dt(timestep),
          v_max(0.22), v_min(-0.05), w_max(2.84),
          q_x(150.0), q_y(150.0), q_theta(80.0),
          r_v(50.0), r_w(50.0),
          terminal_scale(5.0),
          solver_initialized_(false)
    {}

    struct MPCSolution {
        Eigen::Vector2d u;                    // optimal first control [v, omega]
        std::vector<Eigen::Vector3d> pred_traj; // predicted states x0..xN
    };

    // -------------------------------------------------------
    // Solve the MPC QP.
    // x0: current robot state [px, py, θ]
    // refs: reference trajectory states (length >= N+1 recommended)
    // Returns: optimal [v, ω] for the first control step
    // -------------------------------------------------------
    MPCSolution solve(const Eigen::Vector3d& x0,
                          const std::vector<RefState>& refs)
    {
        if (refs.empty()) return MPCSolution{};

        const int n_x = 3;
        const int n_u = 2;
        const int n_U = n_u * N;   // Total decision variables: stacked controls

        // Build cost matrices
        Eigen::Matrix3d Q   = Eigen::DiagonalMatrix<double,3>(q_x, q_y, q_theta);
        Eigen::Matrix2d R   = Eigen::DiagonalMatrix<double,2>(r_v, r_w);
        Eigen::Matrix3d Q_f = Q * terminal_scale;

        // ---- Step 1: Linearize unicycle model at each reference point ----
        // Discrete-time: z_{k+1} = A_k * z_k + B_k * u_k + d_k
        // A_k, B_k: Jacobians of f(z,u) w.r.t. z and u at (z_ref_k, u_ref_k)
        // d_k: affine offset capturing linearization error (exact nonlinear propagation
        //      minus the linear approximation at the reference point)

        std::vector<Eigen::Matrix3d>           A(N);
        std::vector<Eigen::Matrix<double,3,2>> B(N);
        std::vector<Eigen::Vector3d>           d(N);

        for (int k = 0; k < N; ++k) {
            int idx = std::min(k, (int)refs.size() - 1);
            const RefState& ref = refs[idx];
            double theta = ref.theta;
            double v_r   = ref.v;

            A[k] = Eigen::Matrix3d::Identity();
            A[k](0, 2) = -v_r * std::sin(theta) * dt;
            A[k](1, 2) =  v_r * std::cos(theta) * dt;

            B[k].setZero();
            B[k](0, 0) = std::cos(theta) * dt;
            B[k](1, 0) = std::sin(theta) * dt;
            B[k](2, 1) = dt;

            // Exact nonlinear next state at reference
            Eigen::Vector3d z_ref(ref.x, ref.y, ref.theta);
            Eigen::Vector2d u_ref(ref.v, ref.omega);
            Eigen::Vector3d z_nl;
            z_nl(0) = ref.x + v_r * std::cos(theta) * dt;
            z_nl(1) = ref.y + v_r * std::sin(theta) * dt;
            z_nl(2) = ref.theta + ref.omega * dt;
            d[k] = z_nl - A[k] * z_ref - B[k] * u_ref;
        }

        // ---- Step 2: Build condensed sensitivity matrices ----
        // Full predicted state: X = S_z * x0 + S_U * U + S_d
        // where U = [u_0^T, u_1^T, ..., u_{N-1}^T]^T  (2N x 1)
        //       X = [z_0^T, z_1^T, ..., z_N^T]^T       (3(N+1) x 1)
        //
        // S_z ∈ R^{3(N+1)×3}:  state-to-state propagation from x0
        // S_U ∈ R^{3(N+1)×2N}: input-to-state sensitivity
        // S_d ∈ R^{3(N+1)}:    accumulated affine offsets

        Eigen::MatrixXd S_z = Eigen::MatrixXd::Zero(n_x*(N+1), n_x);
        Eigen::MatrixXd S_U = Eigen::MatrixXd::Zero(n_x*(N+1), n_U);
        Eigen::VectorXd S_d = Eigen::VectorXd::Zero(n_x*(N+1));

        // z_0 = x0 (no input)
        S_z.block(0, 0, n_x, n_x) = Eigen::Matrix3d::Identity();

        // Build row by row
        Eigen::Matrix3d   Phi_k = Eigen::Matrix3d::Identity();
        Eigen::VectorXd   sd_k  = Eigen::VectorXd::Zero(n_x);

        // Store S_U rows for propagation (current step's rows from each input j)
        // We propagate: S_U[k+1][j] = A[k] * S_U[k][j] for j<k,  = B[k] for j=k
        for (int k = 0; k < N; ++k) {
            // Propagate Phi and offset
            Phi_k = A[k] * Phi_k;
            sd_k  = A[k] * sd_k + d[k];

            // Fill row k+1 of S_z and S_d
            S_z.block(n_x*(k+1), 0, n_x, n_x) = Phi_k;
            S_d.segment(n_x*(k+1), n_x) = sd_k;

            // Propagate existing columns of S_U from previous steps and add new column
            // New column j=k: input u_k directly drives z_{k+1} via B[k]
            for (int j = 0; j < k; ++j) {
                // S_U[k+1,j] = A[k] * S_U[k,j]
                Eigen::Matrix<double,3,2> prev_col = S_U.block(n_x*k, n_u*j, n_x, n_u);
                S_U.block(n_x*(k+1), n_u*j, n_x, n_u) = A[k] * prev_col;
            }
            // New direct effect of u_k on z_{k+1}
            S_U.block(n_x*(k+1), n_u*k, n_x, n_u) = B[k];
        }

        // ---- Step 3: Build QP cost matrices ----
        // Cost: J = (S_U*U + E_free)^T * Q_bar * (S_U*U + E_free) + U^T * R_bar * U
        // where E_free = S_z * x0 + S_d - X_ref  (free evolution error)
        //
        // Expanding: J = U^T * (S_U^T Q_bar S_U + R_bar) * U + 2 * E_free^T Q_bar S_U * U + const
        //
        // QP: minimize 0.5 * U^T * H * U + f^T * U
        //   H = S_U^T * Q_bar * S_U + R_bar
        //   f = S_U^T * Q_bar * E_free

        // Build block-diagonal Q_bar (3(N+1) x 3(N+1))
        Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(n_x*(N+1), n_x*(N+1));
        for (int k = 1; k < N; ++k)
            Q_bar.block(n_x*k, n_x*k, n_x, n_x) = Q;
        Q_bar.block(n_x*N, n_x*N, n_x, n_x) = Q_f;
        // Note: k=0 block is zero (no cost on z_0 = current state)

        // Build block-diagonal R_bar (2N x 2N)
        Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_U, n_U);
        for (int k = 0; k < N; ++k)
            R_bar.block(n_u*k, n_u*k, n_u, n_u) = R;

        // Build X_ref (reference trajectory as stacked vector)
        Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(n_x*(N+1));
        X_ref.segment(0, n_x) = x0;  // k=0: no error term
        for (int k = 1; k <= N; ++k) {
            int idx = std::min(k, (int)refs.size() - 1);
            X_ref(n_x*k + 0) = refs[idx].x;
            X_ref(n_x*k + 1) = refs[idx].y;
            X_ref(n_x*k + 2) = refs[idx].theta;
        }

        // Free evolution error
        Eigen::VectorXd E_free = S_z * x0 + S_d - X_ref;

        // QP matrices
        Eigen::MatrixXd StQ = S_U.transpose() * Q_bar;
        Eigen::MatrixXd H = StQ * S_U + R_bar;
        Eigen::VectorXd f_vec = StQ * E_free;

        // Symmetrize H (prevents numerical asymmetry from floating-point errors)
        H = 0.5 * (H + H.transpose());

        // ---- Step 4: Set up OSQP ----
        // Build sparse upper-triangular P (OSQP convention)
        Eigen::SparseMatrix<double> P_sparse(n_U, n_U);
        {
            std::vector<Eigen::Triplet<double>> trips;
            trips.reserve(n_U * n_U);
            for (int i = 0; i < n_U; ++i)
                for (int j = i; j < n_U; ++j)
                    if (std::abs(H(i,j)) > 1e-12)
                        trips.push_back({i, j, H(i,j)});
            P_sparse.setFromTriplets(trips.begin(), trips.end());
            P_sparse.makeCompressed();
        }

        // Constraint matrix = Identity (box constraints)
        Eigen::SparseMatrix<double> A_con(n_U, n_U);
        A_con.setIdentity();
        A_con.makeCompressed();

        // Bounds: [v_min, -w_max] <= u_k <= [v_max, w_max]
        Eigen::VectorXd lb(n_U), ub(n_U);
        for (int k = 0; k < N; ++k) {
            lb(2*k)     = v_min;   ub(2*k)   = v_max;
            lb(2*k + 1) = -w_max;  ub(2*k+1) = w_max;
        }

        // Initialize or warm-start OSQP
        if (!solver_initialized_) {
            solver_.settings()->setVerbosity(false);
            solver_.settings()->setWarmStart(true);
            solver_.settings()->setMaxIteration(1000);
            solver_.settings()->setAbsoluteTolerance(1e-4);
            solver_.settings()->setRelativeTolerance(1e-4);

            solver_.data()->setNumberOfVariables(n_U);
            solver_.data()->setNumberOfConstraints(n_U);
            if (!solver_.data()->setHessianMatrix(P_sparse))    return MPCSolution{};
            if (!solver_.data()->setGradient(f_vec))            return MPCSolution{};
            if (!solver_.data()->setLinearConstraintsMatrix(A_con)) return MPCSolution{};
            if (!solver_.data()->setLowerBound(lb))             return MPCSolution{};
            if (!solver_.data()->setUpperBound(ub))             return MPCSolution{};

            if (!solver_.initSolver()) return MPCSolution{};
            solver_initialized_ = true;
        } else {
            // Warm-start: update values, keep primal/dual from previous solve
            solver_.updateHessianMatrix(P_sparse);
            solver_.updateGradient(f_vec);
            solver_.updateBounds(lb, ub);
        }

        // ---- Step 5: Solve ----
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return MPCSolution{};

        Eigen::VectorXd U_opt = solver_.getSolution();
        // return Eigen::Vector2d(U_opt(0), U_opt(1));  // Return u_0 = [v*, ω*]
        // Forward-simulate predicted trajectory using optimal controls
        // X_pred = S_z * x0 + S_U * U_opt + S_d
        Eigen::VectorXd X_pred_flat = S_z * x0 + S_U * U_opt + S_d;

        MPCSolution sol;
        sol.u = Eigen::Vector2d(U_opt(0), U_opt(1));

        // Extract N+1 states from flat vector
        for (int k = 0; k <= N; ++k) {
            sol.pred_traj.push_back(X_pred_flat.segment<3>(3 * k));
        }

        return sol;
    }

    // Reset the solver (call when trajectory changes significantly)
    void reset() {
        if (solver_initialized_) {
            solver_.clearSolver();
            solver_.data()->clearHessianMatrix();
            solver_.data()->clearLinearConstraintsMatrix();
        }
        solver_initialized_ = false;
    }

private:
    OsqpEigen::Solver solver_;
    bool solver_initialized_;
};