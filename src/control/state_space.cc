#include "state_space.h"
#include "ode.h"
#include "linspace.h"

StateSpace::StateSpace(const Eigen::MatrixXd& A,
                       const Eigen::MatrixXd& B,
                       const Eigen::MatrixXd& C,
                       const Eigen::MatrixXd& D)
    : A_(A), B_(B), C_(C), D_(D)
{
    int n = A_.rows();
    if (A_.cols() != n) {
        throw std::invalid_argument("A must be square");
    }
    if (B_.rows() != n || B_.cols() != 1) {
        throw std::invalid_argument("B must be n×1");
    }
    if (C_.rows() != 1 || C_.cols() != n) {
        throw std::invalid_argument("C must be 1×n");
    }
    if (D_.rows() != 1 || D_.cols() != 1) {
        throw std::invalid_argument("D must be 1×1");
    }
}

Eigen::VectorXd StateSpace::derivative(const Eigen::VectorXd& x, double u) const {
    return A_ * x + B_ * u;
}

double StateSpace::output(const Eigen::VectorXd& x, double u) const {
    return (C_ * x + D_ * u)(0, 0);
}

Eigen::VectorXd StateSpace::simulateStep(double u, const Eigen::VectorXd& time, const Eigen::VectorXd& x0) const {
    int nSteps = static_cast<int>(time.size());
    int n = order();
    
    // Result vector for outputs
    Eigen::VectorXd y_output(nSteps);
    
    // Current state
    Eigen::VectorXd x = x0;
    
    // ODESolver for RK4
    ODESolver solver;
    
    // Lambda for the state-space ODE: dx/dt = A*x + B*u
    // Note: u is constant (zero-order hold)
    auto f = [this, u](double t, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return derivative(x, u);
    };
    
    // Initial condition
    y_output(0) = output(x, u);
    
    // Simulate forward in time
    for (int i = 1; i < nSteps; ++i) {
        double stepSize = time(i) - time(i - 1);
        x = solver.rk4Step(f, time(i - 1), x, stepSize);
        y_output(i) = output(x, u);
    }
    
    return y_output;
}
