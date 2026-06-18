#include <gtest/gtest.h>
#include "linspace.h"

TEST(LinspaceTest, TestEndpointsAndCount) {
    Eigen::VectorXd v = linspace(0, 10, 50);

    ASSERT_EQ(v.size(), 50);
    ASSERT_NEAR(v(0), 0.0, 1e-12);
    ASSERT_NEAR(v(49), 10.0, 1e-12);
}

TEST(LinspaceTest, TestUniformSpacing) {
    double start = 0.0;
    double end = 10.0;
    int count = 50;
    Eigen::VectorXd v = linspace(start, end, count);
    double expectedStep = (end-start) / (count-1);

    for (int i = 1; i < v.size(); ++i) {
        ASSERT_NEAR(v(i) - v(i - 1), expectedStep, 1e-12);
    }
}

TEST(LinspaceTest, TestSinglePoint) {
    Eigen::VectorXd v = linspace(3, 7, 1);

    ASSERT_EQ(v.size(), 1);
    ASSERT_NEAR(v(0), 3.0, 1e-12);
}
