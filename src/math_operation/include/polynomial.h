#pragma once
#include <Eigen/Dense>
#include <complex>

double polynomialEval(const Eigen::VectorXd& coeffs, double x);
std::complex<double> polynomialEval(const Eigen::VectorXd& coeffs, std::complex<double> x);
Eigen::VectorXcd polynomialRoots(const Eigen::VectorXd& coeffs);
