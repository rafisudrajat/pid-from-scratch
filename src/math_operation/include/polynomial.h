#pragma once
#include <Eigen/Dense>
#include <complex>

/**
 * @brief Evaluate a polynomial at a real point using Horner's method.
 *
 * Coefficients are in descending power order: for p(x) = x^2 - 3x + 2,
 * pass coeffs = [1, -3, 2].
 *
 * @param coeffs Polynomial coefficients (highest degree first).
 * @param x      Point at which to evaluate.
 * @return p(x).
 */
double polynomialEval(const Eigen::VectorXd& coeffs, double x);

/**
 * @brief Evaluate a polynomial at a complex point using Horner's method.
 *
 * @param coeffs Polynomial coefficients (highest degree first).
 * @param x      Complex point at which to evaluate.
 * @return p(x) as a complex number.
 */
std::complex<double> polynomialEval(const Eigen::VectorXd& coeffs, std::complex<double> x);

/**
 * @brief Find all roots of a polynomial via the companion matrix method.
 *
 * Builds the companion matrix of the monic-normalized polynomial and
 * returns its eigenvalues. Works for any degree >= 1.
 *
 * @param coeffs Polynomial coefficients (highest degree first, leading
 *               coefficient must be non-zero).
 * @return Complex vector of roots (length = degree of the polynomial).
 */
Eigen::VectorXcd polynomialRoots(const Eigen::VectorXd& coeffs);
