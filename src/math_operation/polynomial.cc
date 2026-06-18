#include "polynomial.h"
#include <Eigen/Eigenvalues>

double polynomialEval(const Eigen::VectorXd& coeffs, double x) {
    double result = coeffs(0);
    for (int i = 1; i < coeffs.size(); ++i) {
        result = result * x + coeffs(i);
    }
    return result;
}

std::complex<double> polynomialEval(const Eigen::VectorXd& coeffs, std::complex<double> x) {
    std::complex<double> result = coeffs(0);
    for (int i = 1; i < coeffs.size(); ++i) {
        result = result * x + coeffs(i);
    }
    return result;
}

Eigen::VectorXcd polynomialRoots(const Eigen::VectorXd& coeffs) {
    int n = coeffs.size() - 1; // Degree of the polynomial

    // Step 1: Normalize to monic polynomial
    Eigen::VectorXd monic = coeffs / coeffs(0);

    // Step 2: Build the companion matrix
    Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n - 1; ++i) {
        companion(i, i + 1) = 1.0; // Subdiagonal = 1
    }
    for (int i = 0; i < n; ++i) {
        companion(n - 1, i) = -monic(n - i); // Last row = -monic coefficients
    }

    // Step 3: Compute eigenvalues (roots)
    Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
    return solver.eigenvalues();
}
