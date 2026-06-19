#include <gtest/gtest.h>
#include <cmath>
#include "run_demo.h"
#include "transfer_function.h"

TEST(AppSmokeTest, DemoPipelineRuns) {
    // Smoke test: run the full pipeline on a simple stable first-order plant
    // G = 1/(s + 1), with PI control (Kp=2, Ki=1).
    // The pipeline should complete without error and return finite results
    // whose output length matches the time-grid length.

    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;
    TransferFunction plant(num, den);

    double kp = 2.0;
    double ki = 1.0;
    double kd = 0.0;
    double setpoint = 1.0;
    double tEnd = 10.0;
    int nSteps = 1000;

    DemoResult result = runDemo(plant, kp, ki, kd, setpoint, tEnd, nSteps);

    // Output length must match the time-grid length
    EXPECT_EQ(result.simulation.time.size(), nSteps);
    EXPECT_EQ(result.simulation.output.size(), nSteps);
    EXPECT_EQ(result.simulation.control.size(), nSteps);

    // All output values must be finite (no NaN or inf)
    for (int i = 0; i < nSteps; ++i) {
        EXPECT_TRUE(std::isfinite(result.simulation.output(i)))
            << "output(" << i << ") is not finite";
        EXPECT_TRUE(std::isfinite(result.simulation.control(i)))
            << "control(" << i << ") is not finite";
    }

    // Metrics should be finite for a stable closed loop
    EXPECT_TRUE(std::isfinite(result.metrics.steadyStateValue));
    EXPECT_TRUE(std::isfinite(result.metrics.steadyStateError));
    EXPECT_TRUE(std::isfinite(result.metrics.percentOvershoot));
    EXPECT_TRUE(std::isfinite(result.metrics.riseTime));
    EXPECT_TRUE(std::isfinite(result.metrics.settlingTime));

    // For a stable PI controller on a first-order plant,
    // output should settle near setpoint
    EXPECT_NEAR(result.metrics.steadyStateValue, setpoint, 0.05);
}

TEST(AppSmokeTest, UnstablePlantDiverges) {
    // Faithfulness check: an unstable plant (right-half-plane pole)
    // should produce diverging output, confirming the demo reflects
    // real dynamics rather than a hard-coded curve.
    //
    // G = 1/(s - 1) has a pole at s = +1 (unstable).
    // With only Kp = 0.5 the closed-loop pole remains in the RHP,
    // so the output blows up.

    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, -1.0;
    TransferFunction plant(num, den);

    // Small P-only gain — not enough to stabilize
    double kp = 0.5;
    double ki = 0.0;
    double kd = 0.0;
    double setpoint = 1.0;
    double tEnd = 20.0;
    int nSteps = 1000;

    DemoResult result = runDemo(plant, kp, ki, kd, setpoint, tEnd, nSteps);

    // The output should diverge — the final value should be far from setpoint
    double finalOutput = std::abs(
        result.simulation.output(result.simulation.output.size() - 1));
    EXPECT_GT(finalOutput, 100.0)
        << "Unstable plant should diverge, but output stayed bounded";
}
