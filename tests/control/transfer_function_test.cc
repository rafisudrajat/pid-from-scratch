#include <gtest/gtest.h>
#include "transfer_function.h"

TEST(TransferFunctionTest, DcGainFirstOrder) {
    // G = 5/(2s + 1) => num = [5], den = [2, 1]
    // dcGain = G(0) = b0/a0 = 5/1 = 5
    Eigen::VectorXd num(1);
    num << 5.0;
    Eigen::VectorXd den(2);
    den << 2.0, 1.0;

    TransferFunction tf(num, den);
    ASSERT_NEAR(tf.dcGain(), 5.0, 1e-9);
}

TEST(TransferFunctionTest, DcGainGeneral) {
    // G = (s + 3)/(s^2 + 2s + 6) => num = [1, 3], den = [1, 2, 6]
    // dcGain = G(0) = 3/6 = 0.5
    Eigen::VectorXd num(2);
    num << 1.0, 3.0;
    Eigen::VectorXd den(3);
    den << 1.0, 2.0, 6.0;

    TransferFunction tf(num, den);
    ASSERT_NEAR(tf.dcGain(), 0.5, 1e-9);
}

TEST(TransferFunctionTest, Properness) {
    // Strictly proper: deg(num) < deg(den)
    // G = 5/(2s + 1) => num size 1, den size 2
    Eigen::VectorXd num1(1);
    num1 << 5.0;
    Eigen::VectorXd den1(2);
    den1 << 2.0, 1.0;
    TransferFunction strictlyProper(num1, den1);
    EXPECT_TRUE(strictlyProper.isProper());
    EXPECT_TRUE(strictlyProper.isStrictlyProper());

    // Proper (equal degree): G = (s + 1)/(s + 2)
    // num = [1, 1], den = [1, 2]
    Eigen::VectorXd num2(2);
    num2 << 1.0, 1.0;
    Eigen::VectorXd den2(2);
    den2 << 1.0, 2.0;
    TransferFunction proper(num2, den2);
    EXPECT_TRUE(proper.isProper());
    EXPECT_FALSE(proper.isStrictlyProper());

    // Improper: deg(num) > deg(den)
    // num = [1, 0, 1] (s^2 + 1), den = [1, 2] (s + 2)
    Eigen::VectorXd num3(3);
    num3 << 1.0, 0.0, 1.0;
    Eigen::VectorXd den3(2);
    den3 << 1.0, 2.0;
    TransferFunction improper(num3, den3);
    EXPECT_FALSE(improper.isProper());
    EXPECT_FALSE(improper.isStrictlyProper());
}

TEST(TransferFunctionTest, RejectsZeroDenominator) {
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 0.0, 0.0;

    EXPECT_THROW(TransferFunction(num, den), std::invalid_argument);
}
