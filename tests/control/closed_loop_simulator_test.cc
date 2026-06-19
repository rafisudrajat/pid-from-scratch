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
    double expected_first_output = plant.output(x0, 0.0);
    EXPECT_NEAR(result.output(0), expected_first_output, 1e-12);
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
    double u_open_loop = 0.0;  // offset is 0
    Eigen::VectorXd openLoopOutput = plant.simulateStep(u_open_loop, time, x0);

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
