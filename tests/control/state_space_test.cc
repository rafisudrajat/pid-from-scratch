#include <gtest/gtest.h>
#include "state_space.h"
#include "transfer_function.h"
#include "eigen_matchers.h"

TEST(StateSpaceTest, DerivativeAndOutput) {
    // A = [[0, 1], [-2, -3]], B = [[0], [1]], C = [[1, 0]], D = [[0]]
    // x = [1, 1], u = 1
    // derivative = A*x + B*u = [1, -5] + [0, 1] = [1, -4]
    // output     = C*x + D*u = 1 + 0 = 1
    Eigen::MatrixXd A(2, 2);
    A << 0, 1,
        -2, -3;
    Eigen::MatrixXd B(2, 1);
    B << 0, 1;
    Eigen::MatrixXd C(1, 2);
    C << 1, 0;
    Eigen::MatrixXd D(1, 1);
    D << 0;

    StateSpace ss(A, B, C, D);

    Eigen::VectorXd x(2);
    x << 1.0, 1.0;
    double u = 1.0;

    Eigen::VectorXd dx = ss.derivative(x, u);
    ASSERT_EQ(dx.size(), 2);
    EXPECT_NEAR(dx(0), 1.0, 1e-12);
    EXPECT_NEAR(dx(1), -4.0, 1e-12);

    double y = ss.output(x, u);
    EXPECT_NEAR(y, 1.0, 1e-12);
}

TEST(StateSpaceTest, OutputWithNonZeroD) {
    // Same system but D = [[0.5]]
    // output = C*x + D*u = 1 + 0.5 = 1.5
    Eigen::MatrixXd A(2, 2);
    A << 0, 1,
        -2, -3;
    Eigen::MatrixXd B(2, 1);
    B << 0, 1;
    Eigen::MatrixXd C(1, 2);
    C << 1, 0;
    Eigen::MatrixXd D(1, 1);
    D << 0.5;

    StateSpace ss(A, B, C, D);

    Eigen::VectorXd x(2);
    x << 1.0, 1.0;
    double u = 1.0;

    double y = ss.output(x, u);
    EXPECT_NEAR(y, 1.5, 1e-12);
}

TEST(StateSpaceTest, DimensionValidation) {
    Eigen::MatrixXd A(2, 2);
    A << 0, 1, -2, -3;
    Eigen::MatrixXd B(2, 1);
    B << 0, 1;
    Eigen::MatrixXd C(1, 2);
    C << 1, 0;
    Eigen::MatrixXd D(1, 1);
    D << 0;

    // Non-square A
    Eigen::MatrixXd badA(2, 3);
    badA.setZero();
    EXPECT_THROW(StateSpace(badA, B, C, D), std::invalid_argument);

    // B row count != A rows
    Eigen::MatrixXd badB(3, 1);
    badB.setZero();
    EXPECT_THROW(StateSpace(A, badB, C, D), std::invalid_argument);

    // B not single column (not SISO)
    Eigen::MatrixXd badB2(2, 2);
    badB2.setZero();
    EXPECT_THROW(StateSpace(A, badB2, C, D), std::invalid_argument);

    // C column count != A rows
    Eigen::MatrixXd badC(1, 3);
    badC.setZero();
    EXPECT_THROW(StateSpace(A, B, badC, D), std::invalid_argument);

    // C not single row (not SISO)
    Eigen::MatrixXd badC2(2, 2);
    badC2.setZero();
    EXPECT_THROW(StateSpace(A, B, badC2, D), std::invalid_argument);

    // D not 1x1
    Eigen::MatrixXd badD(2, 1);
    badD.setZero();
    EXPECT_THROW(StateSpace(A, B, C, badD), std::invalid_argument);
}

TEST(StateSpaceTest, EigenvaluesEqualPoles) {
    // G = 1/(s^2 + 3s + 2) => poles {-1, -2}
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 3.0, 2.0;

    TransferFunction tf(num, den);
    StateSpace ss = tf.toStateSpace();

    Eigen::EigenSolver<Eigen::MatrixXd> solver(ss.getA());
    Eigen::VectorXcd eigvals = solver.eigenvalues();

    EXPECT_TRUE(expectComplexSetNear(eigvals, tf.poles(), 1e-9));
}

TEST(StateSpaceTest, RealizationDcGain) {
    // Invariant: -C * A^{-1} * B + D == G.dcGain()

    // Second-order: G = 1/(s^2 + 3s + 2), dcGain = 1/2 = 0.5
    {
        Eigen::VectorXd num(1);
        num << 1.0;
        Eigen::VectorXd den(3);
        den << 1.0, 3.0, 2.0;
        TransferFunction tf(num, den);
        StateSpace ss = tf.toStateSpace();

        double ssGain = (-ss.getC() * ss.getA().inverse() * ss.getB()
                         + ss.getD())(0, 0);
        EXPECT_NEAR(ssGain, tf.dcGain(), 1e-9);
    }

    // First-order: G = 5/(2s + 1), dcGain = 5/1 = 5
    {
        Eigen::VectorXd num(1);
        num << 5.0;
        Eigen::VectorXd den(2);
        den << 2.0, 1.0;
        TransferFunction tf(num, den);
        StateSpace ss = tf.toStateSpace();

        double ssGain = (-ss.getC() * ss.getA().inverse() * ss.getB()
                         + ss.getD())(0, 0);
        EXPECT_NEAR(ssGain, tf.dcGain(), 1e-9);
    }
}

TEST(StateSpaceTest, FirstOrderLayout) {
    // G = K/(tau*s + 1) with K=5, tau=2
    // Realizes to A = [-1/tau], B = [1], C = [K/tau], D = [0]
    double K = 5.0;
    double tau = 2.0;

    Eigen::VectorXd num(1);
    num << K;
    Eigen::VectorXd den(2);
    den << tau, 1.0;

    TransferFunction tf(num, den);
    StateSpace ss = tf.toStateSpace();

    EXPECT_EQ(ss.order(), 1);
    EXPECT_NEAR(ss.getA()(0, 0), -1.0 / tau, 1e-9);
    EXPECT_NEAR(ss.getB()(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(ss.getC()(0, 0), K / tau, 1e-9);
    EXPECT_NEAR(ss.getD()(0, 0), 0.0, 1e-9);
}
