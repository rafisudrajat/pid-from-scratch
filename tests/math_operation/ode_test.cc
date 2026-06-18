#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <functional>
#include "Eigen/Dense"
#include "ode.h"
#include "eigen_matchers.h"

std::vector<double> preComputedResults =
    {1.00000, 3.277344, 3.101563, 2.347656, 2.140625, 2.855469, 4.117188, 4.800781, 3.031250};

double odeDummyFunction(double x, double y){
    double result = -2*std::pow(x,3) + 12*std::pow(x,2) - 20*x + 8.5;
    return result;
}

TEST(RalstonMethodTest, CompareWithPreComputedResults){
    // Define initial value of x0 and y0
    double x0 = 0.0;
    double odeSolverResult = 1.0;
    // Define step size (h)
    double stepSize = 0.5;
    // Create an instance of ODESolver and call the method
    ODESolver solver;
    for(int i=0; i<preComputedResults.size(); i++){
        double x = x0 + stepSize*i;
        ASSERT_NEAR(preComputedResults[i], odeSolverResult, 0.0001);
        odeSolverResult = solver.solveODEWithRalstonMethod(&odeDummyFunction, x, odeSolverResult, stepSize);
    }
}

// --- RK4 vector-valued integrator tests ---

TEST(RK4StepTest, ExactOnCubic) {
    // dx/dt = 3t^2, x(0) = 0  =>  x(t) = t^3
    // RK4 reduces to Simpson's rule for time-only slopes => exact for cubics.
    // One step h = 0.5: x(0.5) = 0.125 exactly.
    auto f = [](double t, const Eigen::VectorXd& /*x*/) -> Eigen::VectorXd {
        Eigen::VectorXd dxdt(1);
        dxdt(0) = 3.0 * t * t;
        return dxdt;
    };

    Eigen::VectorXd x0(1);
    x0(0) = 0.0;

    ODESolver solver;
    Eigen::VectorXd x1 = solver.rk4Step(f, 0.0, x0, 0.5);

    ASSERT_NEAR(x1(0), 0.125, 1e-12);
}

TEST(RK4StepTest, MatchesExponential) {
    // dx/dt = x, x(0) = 1  =>  x(t) = e^t
    // Integrate to t = 1 with small steps => x(1) ≈ e
    auto f = [](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return x;
    };

    Eigen::VectorXd x(1);
    x(0) = 1.0;
    double t = 0.0;
    double h = 0.001;
    int nSteps = static_cast<int>(1.0 / h);

    ODESolver solver;
    for (int i = 0; i < nSteps; ++i) {
        x = solver.rk4Step(f, t, x, h);
        t += h;
    }

    ASSERT_NEAR(x(0), std::exp(1.0), 1e-4);
}

TEST(RK4StepTest, FourthOrderConvergence) {
    // dx/dt = x, x(0) = 1 => x(t) = e^t
    // Integrate to t_end with step h, then again with h/2.
    // Global error ratio should be ≈ 16 (2^4), within 10%.
    auto f = [](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return x;
    };

    double tEnd = 1.0;
    double exact = std::exp(tEnd);
    ODESolver solver;

    auto integrate = [&](double h) -> double {
        Eigen::VectorXd x(1);
        x(0) = 1.0;
        double t = 0.0;
        int nSteps = static_cast<int>(std::round(tEnd / h));
        for (int i = 0; i < nSteps; ++i) {
            x = solver.rk4Step(f, t, x, h);
            t += h;
        }
        return std::abs(x(0) - exact);
    };

    double h = 0.1;
    double errCoarse = integrate(h);
    double errFine = integrate(h / 2.0);

    double ratio = errCoarse / errFine;
    EXPECT_NEAR(ratio, 16.0, 16.0 * 0.10);
}

TEST(RK4StepTest, VectorState) {
    // 2-D system: dx/dt = A*x, x(0) = [1, 0]
    // A = [[0, 1], [-1, 0]]  (harmonic oscillator)
    // Exact: x1(t) = cos(t), x2(t) = -sin(t)
    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
        -1.0, 0.0;

    auto f = [&A](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return A * x;
    };

    Eigen::VectorXd x(2);
    x << 1.0, 0.0;
    double t = 0.0;
    double h = 0.001;
    double tEnd = 2.0;
    int nSteps = static_cast<int>(std::round(tEnd / h));

    ODESolver solver;
    for (int i = 0; i < nSteps; ++i) {
        x = solver.rk4Step(f, t, x, h);
        t += h;
    }

    Eigen::VectorXd expected(2);
    expected << std::cos(tEnd), -std::sin(tEnd);

    EXPECT_TRUE(expectVectorNear(x, expected, 1e-4));
}