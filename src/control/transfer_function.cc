#include "transfer_function.h"
#include "state_space.h"
#include "polynomial.h"
#include <cmath>
#include <limits>

TransferFunction::TransferFunction(const Eigen::VectorXd& numerator,
                                   const Eigen::VectorXd& denominator)
    : numerator_(numerator), denominator_(denominator)
{
    if (denominator_.size() == 0 || denominator_.isZero()) {
        throw std::invalid_argument("Denominator must be non-empty and non-zero");
    }
}

double TransferFunction::dcGain() const {
    double a0 = denominator_(denominator_.size() - 1);
    double b0 = numerator_(numerator_.size() - 1);

    // Type 0 (a_0 != 0): the ordinary finite ratio of the constant terms.
    if (systemType() == 0) {
        return b0 / a0;
    }

    // Type 1+ (a_0 = 0): a pole at the origin makes G(0) unbounded. Report a
    // signed infinity following the sign of b_0 rather than letting IEEE
    // division produce an unsignalled NaN. When b_0 also vanishes the limit
    // is indeterminate from the constant terms alone.
    if (b0 == 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::copysign(std::numeric_limits<double>::infinity(), b0);
}

int TransferFunction::systemType() const {
    // System type = number of poles at the origin = number of trailing zero
    // coefficients in the descending-power denominator (a_0 = 0, a_1 = 0, ...).
    // Compare against a tolerance scaled by the largest coefficient so that
    // round-off in a constructed denominator is not mistaken for an integrator.
    double scale = denominator_.cwiseAbs().maxCoeff();
    double tol = 1e-12 * (scale > 0.0 ? scale : 1.0);

    int type = 0;
    for (int i = static_cast<int>(denominator_.size()) - 1; i >= 0; --i) {
        if (std::abs(denominator_(i)) <= tol) {
            ++type;
        } else {
            break;
        }
    }
    return type;
}

bool TransferFunction::isProper() const {
    return numerator_.size() <= denominator_.size();
}

bool TransferFunction::isStrictlyProper() const {
    return numerator_.size() < denominator_.size();
}

Eigen::VectorXcd TransferFunction::poles() const {
    return polynomialRoots(denominator_);
}

Eigen::VectorXcd TransferFunction::zeros() const {
    return polynomialRoots(numerator_);
}

StateSpace TransferFunction::toStateSpace() const {
    int n = static_cast<int>(denominator_.size()) - 1;

    double leadCoeff = denominator_(0);
    Eigen::VectorXd denNorm = denominator_ / leadCoeff;
    Eigen::VectorXd numNorm = numerator_ / leadCoeff;

    // A: companion matrix — superdiagonal ones, bottom row −a_i (ascending)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n - 1; ++i) {
        A(i, i + 1) = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        A(n - 1, i) = -denNorm(n - i);
    }

    // B: [0 ... 0  1]^T
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, 1);
    B(n - 1, 0) = 1.0;

    // C: [b_0, b_1, ..., b_{n-1}] ascending from normalized numerator
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(1, n);
    int numLen = static_cast<int>(numNorm.size());
    for (int i = 0; i < numLen && i < n; ++i) {
        C(0, i) = numNorm(numLen - 1 - i);
    }

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

    return StateSpace(A, B, C, D);
}

bool TransferFunction::isStable() const {
    Eigen::VectorXcd p = poles();
    for (int i = 0; i < p.size(); ++i) {
        if (p(i).real() >= 0.0) {
            return false;
        }
    }
    return true;
}
