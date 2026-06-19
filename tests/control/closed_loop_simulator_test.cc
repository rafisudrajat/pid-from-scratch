#include <gtest/gtest.h>
#include <cmath>
#include "closed_loop_simulator.h"
#include "state_space.h"
#include "transfer_function.h"
#include "pid_controller.h"
#include "linspace.h"
#include "eigen_matchers.h"

TEST(ClosedLoopSimulatorTest, ResultShapes) {
    // Create a simple first-order plant: G = 1/(s + 1)
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();

    // Create a PID controller with reasonable gains
    PIDController controller(1.0, 0.5, 0.1);

    // Time grid
    double tStart = 0.0;
    double tEnd = 5.0;
    int nSteps = 100;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);

    // Initial state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());

    // Setpoint
    double setpoint = 1.0;

    // Run simulation
    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    // Check shapes
    EXPECT_EQ(result.time.size(), nSteps);
    EXPECT_EQ(result.output.size(), nSteps);
    EXPECT_EQ(result.control.size(), nSteps);

    // First recorded output should be C * x0 (since u[-1] = 0 and D = 0)
    double expectedFirstOutput = plant.output(x0, 0.0);
    EXPECT_NEAR(result.output(0), expectedFirstOutput, 1e-12);
}

TEST(ClosedLoopSimulatorTest, SettlesNearSetpointWithPI) {
    // First-order plant: G = 1/(s + 1)
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();

    // PI controller (no derivative)
    PIDController controller(2.0, 1.0, 0.0);

    // Time grid - run long enough
    double tStart = 0.0;
    double tEnd = 20.0;
    int nSteps = 1000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);

    // Initial state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());

    // Setpoint
    double setpoint = 1.0;

    // Run simulation
    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    // Check final value is close to setpoint (within 1%)
    double finalOutput = result.output(result.output.size() - 1);
    double tolerance = 0.01 * std::abs(setpoint);
    EXPECT_NEAR(finalOutput, setpoint, tolerance);
}

TEST(ClosedLoopSimulatorTest, ZeroGainsIsOpenLoop) {
    // First-order plant: G = 2/(s + 1) -> K = 2, tau = 1
    double K = 2.0;
    double tau = 1.0;
    Eigen::VectorXd num(1);
    num << K;
    Eigen::VectorXd den(2);
    den << tau, 1.0;
    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();

    // Controller with all zero gains
    PIDController controller(0.0, 0.0, 0.0, 0.0);

    // Time grid
    double tStart = 0.0;
    double tEnd = 5.0;
    int nSteps = 500;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);

    // Initial state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());

    // Setpoint
    double setpoint = 1.0;

    // Run closed-loop simulation
    SimulationResult closedLoopResult = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    // Run open-loop simulation with constant input = offset
    double uOpenLoop = 0.0;  // offset is 0
    Eigen::VectorXd openLoopOutput = plant.simulateStep(uOpenLoop, time, x0);

    // With zero gains, the closed-loop should behave like open-loop with u = offset
    // The output should match the open-loop simulation
    for (int i = 0; i < nSteps; ++i) {
        EXPECT_NEAR(closedLoopResult.output(i), openLoopOutput(i), 1e-12);
    }

    // Also check that control signal is constant at offset
    for (int i = 0; i < nSteps; ++i) {
        EXPECT_NEAR(closedLoopResult.control(i), 0.0, 1e-12);
    }
}

TEST(ClosedLoopSimulatorTest, POnlySteadyStateOffset) {
    // Closed-form: For G = K/(tau*s + 1) with unity feedback and P-only controller,
    // the closed-loop transfer function is T = (Kp*K)/(tau*s + 1 + Kp*K)
    // Steady-state: yInfinity = Kp*K/(1 + Kp*K)
    // Steady-state error: eInfinity = 1 - yInfinity = 1/(1 + Kp*K)
    // With K = 1, Kp = 9: yInfinity = 9/10 = 0.9, eInfinity = 0.1

    double K = 1.0;
    double tau = 1.0;
    double Kp = 9.0;

    Eigen::VectorXd num(1);
    num << K;
    Eigen::VectorXd den(2);
    den << tau, 1.0;

    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();

    // P-only controller (Ki = Kd = 0)
    PIDController controller(Kp, 0.0, 0.0, 0.0);

    // Time grid - run long enough for steady state (5*tau should be plenty)
    double tStart = 0.0;
    double tEnd = 50.0;  // Very long to ensure steady state
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);

    // Initial state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());

    // Unit step setpoint
    double setpoint = 1.0;

    // Run simulation
    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    // Expected steady-state values
    double expectedYInfinity = Kp * K / (1.0 + Kp * K);  // 9/10 = 0.9
    double expectedEInfinity = 1.0 / (1.0 + Kp * K);     // 1/10 = 0.1

    // Check final output approaches expected value (within 1%)
    double finalOutput = result.output(result.output.size() - 1);
    EXPECT_NEAR(finalOutput, expectedYInfinity, 1e-2);

    // Check steady-state error
    double finalError = std::abs(setpoint - finalOutput);
    EXPECT_NEAR(finalError, expectedEInfinity, 1e-2);
}

TEST(ClosedLoopSimulatorTest, IntegralEliminatesOffset) {
    // With integral action, the steady-state error should go to zero
    double K = 1.0;
    double tau = 1.0;
    double Kp = 9.0;
    double Ki = 1.0;  // Non-zero integral gain

    Eigen::VectorXd num(1);
    num << K;
    Eigen::VectorXd den(2);
    den << tau, 1.0;

    TransferFunction tf(num, den);
    StateSpace plant = tf.toStateSpace();

    // PI controller (with integral action)
    PIDController controller(Kp, Ki, 0.0, 0.0);

    // Time grid - run long enough
    double tStart = 0.0;
    double tEnd = 50.0;
    int nSteps = 10000;
    Eigen::VectorXd time = linspace(tStart, tEnd, nSteps);

    // Initial state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());

    // Unit step setpoint
    double setpoint = 1.0;

    // Run simulation
    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    // With integral action, final error should be very close to zero
    double finalOutput = result.output(result.output.size() - 1);
    double finalError = std::abs(setpoint - finalOutput);

    // Integral action should drive error below 0.1%
    EXPECT_LT(finalError, 1e-3);
}
