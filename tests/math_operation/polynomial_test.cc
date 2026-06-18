#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "polynomial.h"
#include "eigen_matchers.h"

TEST(PolynomialTest, TestEvalHandValues) {
    Eigen::VectorXd coeffs(3);
    coeffs << 1, -3, 2;

    ASSERT_NEAR(polynomialEval(coeffs, 0.0), 2.0, 1e-9);
    ASSERT_NEAR(polynomialEval(coeffs, 1.0), 0.0, 1e-9);
    ASSERT_NEAR(polynomialEval(coeffs, 2.0), 0.0, 1e-9);
}

TEST(PolynomialTest, TestRealRoots) {
    Eigen::VectorXd coeffs(3);
    coeffs << 1, -3, 2;

    Eigen::VectorXcd expected(2);
    expected << std::complex<double>(1, 0), std::complex<double>(2, 0);

    EXPECT_TRUE(expectComplexSetNear(polynomialRoots(coeffs), expected, 1e-9));
}

TEST(PolynomialTest, TestComplexRoots) {
    Eigen::VectorXd coeffs(3);
    coeffs << 1, 0, 1;

    Eigen::VectorXcd expected(2);
    expected << std::complex<double>(0, 1), std::complex<double>(0, -1);

    EXPECT_TRUE(expectComplexSetNear(polynomialRoots(coeffs), expected, 1e-9));
}

TEST(PolynomialTest, TestRootsMatchEval) {
    Eigen::VectorXd coeffs(3);
    coeffs << 1, -3, 2;

    Eigen::VectorXcd roots = polynomialRoots(coeffs);
    for (int i = 0; i < roots.size(); ++i) {
        ASSERT_LT(std::abs(polynomialEval(coeffs, roots(i))), 1e-8)
            << "root " << i << " = " << roots(i) << " is not a root";
    }
}
