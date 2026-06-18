#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <Eigen/Dense>
#include "eigen_matchers.h"

TEST(EigenMatchers, TestVectorNearPassesAndFails) {
    Eigen::VectorXd v(3);
    v << 1, 2, 3;

    // Must pass: identical vectors
    EXPECT_TRUE(expectVectorNear(v, v, 1e-9));

    // Must fail: index 2 differs by 1.0
    Eigen::VectorXd w(3);
    w << 1, 2, 4;
    EXPECT_NONFATAL_FAILURE(
        EXPECT_TRUE(expectVectorNear(v, w, 1e-9)),
        "index 2"
    );
}

TEST(EigenMatchers, TestComplexSetIsOrderInsensitive) {
    Eigen::VectorXcd a(2);
    a << std::complex<double>(1, 0), std::complex<double>(2, 0);

    Eigen::VectorXcd b(2);
    b << std::complex<double>(2, 0), std::complex<double>(1, 0);

    // Same set, different order — must pass
    EXPECT_TRUE(expectComplexSetNear(a, b, 1e-9));

    // Different set — must fail
    Eigen::VectorXcd c(2);
    c << std::complex<double>(1, 0), std::complex<double>(3, 0);
    EXPECT_NONFATAL_FAILURE(
        EXPECT_TRUE(expectComplexSetNear(a, c, 1e-9)),
        ""
    );
}
