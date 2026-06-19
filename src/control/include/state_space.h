#pragma once
#include <Eigen/Dense>
#include <stdexcept>

/**
 * @brief SISO state-space model (A, B, C, D).
 *
 * Encodes the continuous-time state equations
 *   dx/dt = A·x + B·u   (state derivative)
 *   y     = C·x + D·u   (output)
 * where x is the n-dimensional state, u is scalar input,
 * and y is scalar output (SISO).
 */
class StateSpace {
public:
    /**
     * @brief Construct from the four state-space matrices.
     *
     * @param A System matrix (n×n).
     * @param B Input matrix  (n×1).
     * @param C Output matrix (1×n).
     * @param D Feedthrough   (1×1).
     * @throws std::invalid_argument If dimensions are inconsistent.
     */
    StateSpace(const Eigen::MatrixXd& A,
               const Eigen::MatrixXd& B,
               const Eigen::MatrixXd& C,
               const Eigen::MatrixXd& D);

    /**
     * @brief State derivative dx/dt = A·x + B·u.
     */
    Eigen::VectorXd derivative(const Eigen::VectorXd& x, double u) const;

    /**
     * @brief Scalar output y = C·x + D·u.
     */
    double output(const Eigen::VectorXd& x, double u) const;

    /**
     * @brief Simulate the open-loop step response.
     *
     * Simulates dx/dt = A·x + B·u, y = C·x + D·u with constant input u
     * (zero-order hold) over the given time grid using RK4 integration.
     *
     * @param u        Constant input value (unit step).
     * @param time     Time grid (n×1 vector of time points).
     * @param x0       Initial state vector.
     * @return Output y at each time point.
     */
    Eigen::VectorXd simulateStep(double u, const Eigen::VectorXd& time, const Eigen::VectorXd& x0) const;

    const Eigen::MatrixXd& getA() const { return A_; }
    const Eigen::MatrixXd& getB() const { return B_; }
    const Eigen::MatrixXd& getC() const { return C_; }
    const Eigen::MatrixXd& getD() const { return D_; }

    int order() const { return static_cast<int>(A_.rows()); }

private:
    Eigen::MatrixXd A_, B_, C_, D_;
};
