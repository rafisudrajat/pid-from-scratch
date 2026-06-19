#include <gtest/gtest.h>
#include <cmath>
#include "step_response_metrics.h"
#include "state_space.h"
#include "transfer_function.h"
#include "linspace.h"
#include "eigen_matchers.h"

TEST(StepResponseMetricsTest, OvershootOfSecondOrder) {
    // Closed-form oracle: G = 1/(s^2 + s + 1) is a standard second-order system
    // with ζ = 0.5, ωₙ = 1
    // The analytic overshoot is Mp = exp(-πζ/√(1-ζ²)) ≈ 0.163 (16.3%)
    
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();
    
    // Simulate open-loop step response
    double u = 1.0;  // unit step
    double tStart = 0.0;
    double tEnd = 20.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    Eigen::VectorXd output = plant.simulateStep(u, time, x0);
    
    // Compute metrics
    double setpoint = 1.0;  // step input
    StepMetrics metrics = StepResponseMetrics::compute(time, output, setpoint);
    
    // Expected overshoot ≈ 16.3% (0.163 in decimal)
    // Tolerance: 0.5% absolute (0.005)
    EXPECT_NEAR(metrics.percentOvershoot, 16.3, 0.5);
}

TEST(StepResponseMetricsTest, PeakTime) {
    // For the same second-order system G = 1/(s^2 + s + 1)
    // with ζ = 0.5, ωₙ = 1
    // Peak time tp = π/(ωₙ√(1-ζ²)) ≈ 3.6276 s
    
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();
    
    // Simulate open-loop step response
    double u = 1.0;  // unit step
    double tStart = 0.0;
    double tEnd = 20.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    Eigen::VectorXd output = plant.simulateStep(u, time, x0);
    
    // Compute metrics
    double setpoint = 1.0;
    StepMetrics metrics = StepResponseMetrics::compute(time, output, setpoint);
    
    // Expected peak time ≈ 3.63 s, within one grid step (0.002 s)
    EXPECT_NEAR(metrics.peakTime, 3.63, 0.01);
}

TEST(StepResponseMetricsTest, SettlingAndSteadyState) {
    // Test with G = 1/(s^2 + s + 1)
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();
    
    double u = 1.0;
    double tStart = 0.0;
    double tEnd = 50.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    Eigen::VectorXd output = plant.simulateStep(u, time, x0);
    
    double setpoint = 1.0;
    StepMetrics metrics = StepResponseMetrics::compute(time, output, setpoint);
    
    // Steady-state value should be approximately 1
    EXPECT_NEAR(metrics.steadyStateValue, 1.0, 1e-6);
    
    // Steady-state error should be close to 0
    EXPECT_NEAR(metrics.steadyStateError, 0.0, 1e-6);
    
    // Settling time should be finite and past the peak
    EXPECT_GT(metrics.settlingTime, metrics.peakTime);
    EXPECT_GT(metrics.settlingTime, 0.0);
    EXPECT_LT(metrics.settlingTime, tEnd);
    
    // First-order system G = 1/(s + 1) should have no overshoot
    Eigen::VectorXd numFirstOrder(1);
    numFirstOrder << 1.0;
    Eigen::VectorXd denFirstOrder(2);
    denFirstOrder << 1.0, 1.0;
    TransferFunction tfFirstOrder(numFirstOrder, denFirstOrder);
    StateSpace plantFirstOrder = tfFirstOrder.toStateSpace();

    Eigen::VectorXd x0FirstOrder = Eigen::VectorXd::Zero(plantFirstOrder.order());
    Eigen::VectorXd outputFirstOrder = plantFirstOrder.simulateStep(u, time, x0FirstOrder);
    StepMetrics metricsFirstOrder = StepResponseMetrics::compute(time, outputFirstOrder, setpoint);

    // First-order system should have no overshoot
    EXPECT_NEAR(metricsFirstOrder.percentOvershoot, 0.0, 1e-6);
}

TEST(StepResponseMetricsTest, SteadyStateError) {
    // This ties to Phase 5 P-only run where we expect e_ss ≈ 0.1
    // For G = K/(τs + 1) with K = 1, τ = 1, under P-only with Kp = 9
    // The closed-loop steady-state output is y_∞ = Kp*K/(1+Kp*K) = 9/10 = 0.9
    // So error = 1 - 0.9 = 0.1
    
    // We'll use a pre-computed step response that settles at 0.9
    double tStart = 0.0;
    double tEnd = 50.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);
    
    // Create a response that settles at 0.9
    Eigen::VectorXd output = Eigen::VectorXd::Zero(nSteps);
    for (int i = 0; i < nSteps; ++i) {
        output(i) = 0.9 * (1.0 - std::exp(-time(i)));  // Approximate first-order settling at 0.9
    }
    
    double setpoint = 1.0;
    StepMetrics metrics = StepResponseMetrics::compute(time, output, setpoint);
    
    // Expected steady-state error ≈ 0.1
    EXPECT_NEAR(metrics.steadyStateError, 0.1, 1e-2);
    EXPECT_NEAR(metrics.steadyStateValue, 0.9, 1e-2);
}

TEST(StepResponseMetricsTest, RiseTime) {
    // Test rise time for a first-order system G = 1/(s + 1)
    // Rise time (10% to 90%) should be approximately ln(9) ≈ 2.197
    
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();
    
    double u = 1.0;
    double tStart = 0.0;
    double tEnd = 10.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    Eigen::VectorXd output = plant.simulateStep(u, time, x0);
    
    double setpoint = 1.0;
    StepMetrics metrics = StepResponseMetrics::compute(time, output, setpoint);
    
    // Expected rise time ≈ ln(9) ≈ 2.197
    EXPECT_NEAR(metrics.riseTime, std::log(9.0), 0.01);
}
