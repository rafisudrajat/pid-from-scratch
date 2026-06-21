#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "transfer_function.h"
#include "eigen_matchers.h"

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

TEST(TransferFunctionTest, PolesOfSecondOrder) {
    // G = 1/(s^2 + 3s + 2) => den = [1, 3, 2]
    // poles = roots of s^2 + 3s + 2 = (s+1)(s+2) => {-1, -2}
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 3.0, 2.0;

    TransferFunction tf(num, den);
    Eigen::VectorXcd expectedPoles(2);
    expectedPoles << std::complex<double>(-1.0, 0.0),
                     std::complex<double>(-2.0, 0.0);
    EXPECT_TRUE(expectComplexSetNear(tf.poles(), expectedPoles, 1e-9));
}

TEST(TransferFunctionTest, ComplexPoles) {
    // G = 1/(s^2 + 2s + 5) => den = [1, 2, 5]
    // poles = -1 +/- 2i
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 2.0, 5.0;

    TransferFunction tf(num, den);
    Eigen::VectorXcd expectedPoles(2);
    expectedPoles << std::complex<double>(-1.0, 2.0),
                     std::complex<double>(-1.0, -2.0);
    EXPECT_TRUE(expectComplexSetNear(tf.poles(), expectedPoles, 1e-9));
}

TEST(TransferFunctionTest, Zeros) {
    // G = (s^2 - 1)/(s^2 + 2s + 2) => num = [1, 0, -1], den = [1, 2, 2]
    // zeros = roots of s^2 - 1 = (s-1)(s+1) => {-1, +1}
    Eigen::VectorXd num(3);
    num << 1.0, 0.0, -1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 2.0, 2.0;

    TransferFunction tf(num, den);
    Eigen::VectorXcd expectedZeros(2);
    expectedZeros << std::complex<double>(-1.0, 0.0),
                     std::complex<double>(1.0, 0.0);
    EXPECT_TRUE(expectComplexSetNear(tf.zeros(), expectedZeros, 1e-9));
}

TEST(TransferFunctionTest, StabilityPredicate) {
    // Stable: G = 1/(s^2 + 3s + 2), poles {-1, -2} — all Re < 0
    Eigen::VectorXd num1(1);
    num1 << 1.0;
    Eigen::VectorXd den1(3);
    den1 << 1.0, 3.0, 2.0;
    TransferFunction stable(num1, den1);
    EXPECT_TRUE(stable.isStable());

    // Unstable: G = 1/(s^2 - 1), poles {-1, +1} — one Re > 0
    Eigen::VectorXd num2(1);
    num2 << 1.0;
    Eigen::VectorXd den2(3);
    den2 << 1.0, 0.0, -1.0;
    TransferFunction unstable(num2, den2);
    EXPECT_FALSE(unstable.isStable());
}

TEST(TransferFunctionTest, SystemTypeZero) {
    // Type 0: no pole at the origin. G = 5/(2s + 1) => den = [2, 1], a0 = 1 != 0.
    Eigen::VectorXd num(1);
    num << 5.0;
    Eigen::VectorXd den(2);
    den << 2.0, 1.0;

    TransferFunction tf(num, den);
    EXPECT_EQ(tf.systemType(), 0);
}

TEST(TransferFunctionTest, SystemTypeOne) {
    // Type 1: one pole at the origin. G = 1/(s(s+1)) = 1/(s^2 + s)
    // => den = [1, 1, 0]: a0 = 0, a1 = 1 != 0 → exactly one trailing zero.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 0.0;

    TransferFunction tf(num, den);
    EXPECT_EQ(tf.systemType(), 1);
}

TEST(TransferFunctionTest, SystemTypeTwo) {
    // Type 2: double integrator. G = 1/s^2 => den = [1, 0, 0]:
    // a0 = 0 and a1 = 0 → two trailing zeros.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 0.0, 0.0;

    TransferFunction tf(num, den);
    EXPECT_EQ(tf.systemType(), 2);
}

TEST(TransferFunctionTest, DcGainType1IsInfinite) {
    // A type-1 plant has a pole at the origin, so G(0) is unbounded:
    // the open-loop step response integrates without limit. dcGain() must
    // report +infinity (not a silent NaN) when b0 != 0.
    // G = 1/(s^2 + s) => num = [1], den = [1, 1, 0].
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 0.0;

    TransferFunction tf(num, den);
    double g0 = tf.dcGain();
    EXPECT_TRUE(std::isinf(g0));
    EXPECT_GT(g0, 0.0);
}

TEST(TransferFunctionTest, DcGainType1NegativeNumerator) {
    // Sign of the infinity follows the sign of b0. G = -3/(s^2 + s).
    Eigen::VectorXd num(1);
    num << -3.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 0.0;

    TransferFunction tf(num, den);
    double g0 = tf.dcGain();
    EXPECT_TRUE(std::isinf(g0));
    EXPECT_LT(g0, 0.0);
}

TEST(TransferFunctionTest, DcGainType2IsInfinite) {
    // Double integrator G = 1/s^2 also has infinite DC gain.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 0.0, 0.0;

    TransferFunction tf(num, den);
    EXPECT_TRUE(std::isinf(tf.dcGain()));
}
