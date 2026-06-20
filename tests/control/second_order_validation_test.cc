#include <gtest/gtest.h>
#include <cmath>
#include "transfer_function.h"
#include "state_space.h"
#include "pid_controller.h"
#include "closed_loop_simulator.h"
#include "step_response_metrics.h"
#include "linspace.h"
#include "ode.h"

// Helper function to run closed-loop simulation with control computed BEFORE integrating
// This matches the notebook's structure and avoids the huge initial D-term issue
static SimulationResult simulateClosedLoopWithNewControl(
    const StateSpace& plant,
    PIDController& controller,
    double setpoint,
    const Eigen::VectorXd& time,
    const Eigen::VectorXd& x0) {
    
    int nSteps = static_cast<int>(time.size());
    SimulationResult result;
    result.time = time;
    result.output = Eigen::VectorXd::Zero(nSteps);
    result.control = Eigen::VectorXd::Zero(nSteps);
    
    Eigen::VectorXd x = x0;
    ODESolver solver;
    
    // Initialize controller time to avoid huge first D term
    controller.reset();
    controller.setInitialTime(-1e-6);
    
    // First output measurement at t=0 with u=0
    result.control(0) = 0.0;
    result.output(0) = plant.output(x, 0.0);
    
    // Run the loop from k=1 to nSteps-1
    for (int k = 1; k < nSteps; ++k) {
        double current_time = time(k);
        double prev_time = time(k - 1);
        double stepSize = current_time - prev_time;
        
        // Compute u[k] using previous output y[k-1]
        result.control(k) = controller.compute(setpoint, result.output(k - 1), current_time);
        double uCurrent = result.control(k);
        
        // Integrate with the new control value u[k] (not u[k-1])
        auto f = [&plant, uCurrent](double t, const Eigen::VectorXd& xState) -> Eigen::VectorXd {
            return plant.derivative(xState, uCurrent);
        };
        
        x = solver.rk4Step(f, prev_time, x, stepSize);
        
        // Measure y[k] = C * x[k] + D * u[k]
        result.output(k) = plant.output(x, uCurrent);
    }
    
    return result;
}

TEST(SecondOrderValidationTest, ClosedLoopOvershootTracksTheory) {
    // Second-order closed-loop matches analytic metrics
    // Test that a closed-loop system with ζ=0.5, ωₙ=1 gives the expected
    // overshoot and peak time based on analytic formulas.
    //
    // Analytic formulas for second-order system with characteristic polynomial s² + s + 1:
    //   ζ = 0.5, ωₙ = 1
    //   Percent overshoot Mp = exp(-πζ/√(1-ζ²)) ≈ 16.3%
    //   Peak time tp = π/(ωₙ√(1-ζ²)) ≈ 3.63 s
    //
    // To achieve this, we use plant G(s) = 1/(s² + s) = 1/(s(s+1)) with P controller Kp=1.
    // Closed-loop: T(s) = 1 / (s² + s + 1)
    // Characteristic polynomial: s² + s + 1 = 0, which gives exactly ζ=0.5, ωₙ=1.
    //
    // Note: In our polynomial convention (descending powers): numerator [1], denominator [1, 1, 0]
    Eigen::VectorXd numerator(1);
    numerator << 1.0;
    
    Eigen::VectorXd denominator(3);
    denominator << 1.0, 1.0, 0.0;  // s² + s + 0
    
    TransferFunction tf(numerator, denominator);
    
    // Convert to state space
    StateSpace plant = tf.toStateSpace();
    
    // P controller with Kp=1
    // Closed-loop: T(s) = 1 / (s² + s + 1)
    // Characteristic polynomial: s² + s + 1 = 0, which gives ζ=0.5, ωₙ=1
    PIDController controller(1.0, 0.0, 0.0, 0.0);
    
    // Time grid: simulate for 20 seconds with small step size
    const int nSteps = 2000;
    Eigen::VectorXd time = linspace(0.0, 20.0, nSteps);
    
    // Initial state: zero
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    
    // Setpoint
    const double setpoint = 1.0;
    
    // Run closed-loop simulation
    SimulationResult result = simulateClosedLoopWithNewControl(
        plant, controller, setpoint, time, x0);
    
    // Compute step response metrics
    StepMetrics metrics = StepResponseMetrics::compute(result.time, result.output, setpoint);
    
    // Analytic values for closed-loop with characteristic polynomial s² + s + 1
    // G(s) = 1/(s² + s) with P controller Kp=1 gives:
    //   T(s) = 1 / (s² + s + 1)
    //   Characteristic polynomial: s² + s + 1 = 0
    //   ζ = 0.5, ωₙ = 1
    const double zeta = 0.5;
    const double omega_n = 1.0;
    const double analyticOvershoot = std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta)) * 100.0;
    const double analyticPeakTime = M_PI / (omega_n * std::sqrt(1.0 - zeta * zeta));
    
    // Expected values
    const double expectedOvershoot = analyticOvershoot;  // ~16.3%
    const double expectedPeakTime = analyticPeakTime;    // ~3.63 s
    
    // Tolerance: "within a few percent" - use 5% absolute for overshoot, 0.5s for peak time
    const double overshootTolerance = 5.0;  // ±5% absolute
    const double peakTimeTolerance = 0.5;   // ±0.5 s
    
    // Verify overshoot matches analytic prediction
    EXPECT_NEAR(metrics.percentOvershoot, expectedOvershoot, overshootTolerance)
        << "Measured overshoot: " << metrics.percentOvershoot << "%, expected: ~" << expectedOvershoot << "%"
        << " (analytic: " << analyticOvershoot << "%)";
    
    // Verify peak time matches analytic prediction
    EXPECT_NEAR(metrics.peakTime, expectedPeakTime, peakTimeTolerance)
        << "Measured peak time: " << metrics.peakTime << "s, expected: ~" << expectedPeakTime << "s"
        << " (analytic: " << analyticPeakTime << "s)";
    
    // Verify steady-state value is close to setpoint (DC gain = 1)
    EXPECT_NEAR(metrics.steadyStateValue, setpoint, 0.05)
        << "Steady-state value: " << metrics.steadyStateValue << ", expected: " << setpoint;
}

TEST(SecondOrderValidationTest, HigherKdReducesOvershoot) {
    // Test that increasing Kd reduces overshoot
    // Use a fixed second-order plant and increase Kd while keeping Kp and Ki constant.
    // The overshoot should monotonically decrease as Kd increases.
    //
    // Plant: G(s) = 1/(s² + s + 1) - a second-order plant with ζ=0.5, ωₙ=1
    // This is underdamped and will exhibit overshoot with appropriate controller gains.

    // Create transfer function G(s) = 1/(s² + s + 1)
    Eigen::VectorXd numerator(1);
    numerator << 1.0;
    
    Eigen::VectorXd denominator(3);
    denominator << 1.0, 1.0, 1.0;
    
    TransferFunction tf(numerator, denominator);
    StateSpace plant = tf.toStateSpace();
    
    // Time grid
    const int nSteps = 2000;
    Eigen::VectorXd time = linspace(0.0, 30.0, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    const double setpoint = 1.0;
    
    // Test with different Kd values, keeping Kp and Ki constant
    // Use gains that will produce some overshoot with Kd=0
    const double kp = 3.0;
    const double ki = 1.0;
    
    std::vector<double> kdValues = {0.0, 0.5, 1.0, 1.5, 2.0};
    std::vector<double> overshoots;
    
    for (double kd : kdValues) {
        PIDController controller(kp, ki, kd, 0.0);
        
        SimulationResult result = simulateClosedLoopWithNewControl(
            plant, controller, setpoint, time, x0);
        
        StepMetrics metrics = StepResponseMetrics::compute(result.time, result.output, setpoint);
        overshoots.push_back(metrics.percentOvershoot);
    }
    
    // Verify that overshoot monotonically decreases (or at least doesn't increase) as Kd increases
    for (size_t i = 1; i < overshoots.size(); ++i) {
        EXPECT_LE(overshoots[i], overshoots[i-1] + 0.1)  // Allow small tolerance
            << "Overshoot should decrease as Kd increases. "
            << "Kd=" << kdValues[i-1] << " overshoot=" << overshoots[i-1] << "%, "
            << "Kd=" << kdValues[i] << " overshoot=" << overshoots[i] << "%";
    }
}
