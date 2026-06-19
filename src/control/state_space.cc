#include "state_space.h"

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
