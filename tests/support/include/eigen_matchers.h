#pragma once
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

inline ::testing::AssertionResult expectVectorNear(
    const Eigen::VectorXd& a, 
    const Eigen::VectorXd& b, 
    double tol) 
{
    if (a.size() != b.size()) {
        return ::testing::AssertionFailure()
            << "size mismatch: " << a.size() << " vs " << b.size();
    }
    int worstIdx = 0;
    double worstErr = 0.0;
    for (int i = 0; i < a.size(); ++i) {
        double err = std::abs(a(i) - b(i));
        if (err > worstErr) {
            worstErr = err;
            worstIdx = i;
        }
    }
    if (worstErr > tol) {
        return ::testing::AssertionFailure()
            << "vectors differ at index " << worstIdx
            << ": |" << a(worstIdx) << " - " << b(worstIdx)
            << "| = " << worstErr << " > tol " << tol;
    }
    return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult expectMatrixNear(
    const Eigen::MatrixXd& A, 
    const Eigen::MatrixXd& B, 
    double tol) 
{
    if (A.rows() != B.rows() || A.cols() != B.cols()) {
        return ::testing::AssertionFailure()
            << "size mismatch: (" << A.rows() << "x" << A.cols()
            << ") vs (" << B.rows() << "x" << B.cols() << ")";
    }
    int worstRow = 0, worstCol = 0;
    double worstErr = 0.0;
    for (int r = 0; r < A.rows(); ++r) {
        for (int c = 0; c < A.cols(); ++c) {
            double err = std::abs(A(r, c) - B(r, c));
            if (err > worstErr) {
                worstErr = err;
                worstRow = r;
                worstCol = c;
            }
        }
    }
    if (worstErr > tol) {
        return ::testing::AssertionFailure()
            << "matrices differ at (" << worstRow << ", " << worstCol
            << "): |" << A(worstRow, worstCol) << " - " << B(worstRow, worstCol)
            << "| = " << worstErr << " > tol " << tol;
    }
    return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult expectComplexSetNear(
    const Eigen::VectorXcd& a, const Eigen::VectorXcd& b, double tol) {
    if (a.size() != b.size()) {
        return ::testing::AssertionFailure()
            << "size mismatch: " << a.size() << " vs " << b.size();
    }
    auto sortKey = [](const std::complex<double>& x) {
        return std::make_pair(x.real(), x.imag());
    };
    std::vector<std::complex<double>> sa(a.data(), a.data() + a.size());
    std::vector<std::complex<double>> sb(b.data(), b.data() + b.size());
    std::sort(sa.begin(), sa.end(), [&](auto& x, auto& y) {
        return sortKey(x) < sortKey(y);
    });
    std::sort(sb.begin(), sb.end(), [&](auto& x, auto& y) {
        return sortKey(x) < sortKey(y);
    });
    for (int i = 0; i < (int)sa.size(); ++i) {
        double err = std::abs(sa[i] - sb[i]);
        if (err > tol) {
            return ::testing::AssertionFailure()
                << "complex sets differ at sorted index " << i
                << ": " << sa[i] << " vs " << sb[i]
                << ", |diff| = " << err << " > tol " << tol;
        }
    }
    return ::testing::AssertionSuccess();
}
