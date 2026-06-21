#include <gtest/gtest.h>
#include <cmath>
#include "transfer_function.h"
#include "state_space.h"
#include "pid_controller.h"
#include "closed_loop_simulator.h"
#include "step_response_metrics.h"
#include "linspace.h"
#include "ode.h"

// These tests validate the *closed-loop* behavior that makes "system type"
// matter in the first place: the number of integrators (poles at the origin)
// in the open loop sets how many reference shapes can be tracked with zero
// steady-state error. A type-1 plant tracks a step with zero error under pure
// proportional control; a type-0 plant does not. A type-2 plant needs
// derivative action for stability but then also tracks a step with zero error.

// Compute the control BEFORE integrating and seed the controller's previous
// time slightly in the past. This mirrors the notebook reference structure and
// avoids the one-sample derivative "kick" the production first step produces
// when Kd != 0 and Ts collapses to the guard value. (Same helper rationale as
// second_order_validation_test.cc.)
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

    controller.reset();
    controller.setInitialTime(-1e-6);

    result.control(0) = 0.0;
    result.output(0) = plant.output(x, 0.0);

    for (int k = 1; k < nSteps; ++k) {
        double current_time = time(k);
        double prev_time = time(k - 1);
        double stepSize = current_time - prev_time;

        result.control(k) = controller.compute(setpoint, result.output(k - 1), current_time);
        double uCurrent = result.control(k);

        auto f = [&plant, uCurrent](double t, const Eigen::VectorXd& xState) -> Eigen::VectorXd {
            return plant.derivative(xState, uCurrent);
        };

        x = solver.rk4Step(f, prev_time, x, stepSize);
        result.output(k) = plant.output(x, uCurrent);
    }

    return result;
}

TEST(SystemTypeValidationTest, Type1PlantTracksStepWithZeroErrorUnderPControl) {
    // Type-1 plant G(s) = 1/(s(s+1)) = 1/(s^2 + s). The pole at the origin is
    // a built-in integrator, so a pure proportional controller (Kp = 1) drives
    // the steady-state error to zero. Closed loop: T(s) = 1/(s^2 + s + 1),
    // which is stable and settles at the setpoint.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 1.0, 0.0;  // s^2 + s

    TransferFunction tf(num, den);
    ASSERT_EQ(tf.systemType(), 1);

    StateSpace plant = tf.toStateSpace();

    // Pure proportional control — no integral term needed for zero error.
    PIDController controller(1.0, 0.0, 0.0, 0.0);

    const int nSteps = 4000;
    Eigen::VectorXd time = linspace(0.0, 40.0, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    const double setpoint = 1.0;

    // Pure P control has no derivative kick, so the shipped simulator is used.
    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    StepMetrics metrics = StepResponseMetrics::compute(
        result.time, result.output, setpoint);

    EXPECT_NEAR(metrics.steadyStateValue, setpoint, 0.01)
        << "Type-1 plant should track the step. ss = " << metrics.steadyStateValue;
    EXPECT_LT(metrics.steadyStateError, 0.01)
        << "Type-1 plant under P control must have ~zero steady-state error.";
}

TEST(SystemTypeValidationTest, Type0PlantHasNonzeroErrorUnderPControl) {
    // Contrast: a type-0 plant G(s) = 1/(s+1) under the SAME pure proportional
    // controller (Kp = 1) leaves a finite steady-state error. Closed loop:
    // T(s) = 1/(s+2), DC gain 0.5, so the output settles at 0.5 — error 0.5.
    // This is exactly the limitation that an integrator removes.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;  // s + 1

    TransferFunction tf(num, den);
    ASSERT_EQ(tf.systemType(), 0);

    StateSpace plant = tf.toStateSpace();
    PIDController controller(1.0, 0.0, 0.0, 0.0);

    const int nSteps = 2000;
    Eigen::VectorXd time = linspace(0.0, 20.0, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    const double setpoint = 1.0;

    SimulationResult result = ClosedLoopSimulator::simulate(
        plant, controller, setpoint, time, x0);

    StepMetrics metrics = StepResponseMetrics::compute(
        result.time, result.output, setpoint);

    // Output settles at the closed-loop DC gain 0.5, leaving error ≈ 0.5.
    EXPECT_NEAR(metrics.steadyStateValue, 0.5, 0.02);
    EXPECT_GT(metrics.steadyStateError, 0.4)
        << "Type-0 plant under P control must retain a finite steady-state error.";
}

TEST(SystemTypeValidationTest, Type2PlantTracksStepWithPDControl) {
    // Type-2 plant: the double integrator G(s) = 1/s^2. Pure proportional
    // control would leave the loop marginally stable (poles on the jω axis),
    // so derivative action is required. With a PD controller Kp = 1, Kd = 2 the
    // closed-loop poles are s^2 + 2s + 1 = (s+1)^2, and the two integrators
    // give zero steady-state error to a step.
    //
    // Note the closed loop is T(s) = (2s + 1)/(s + 1)^2: the PD controller
    // contributes a ZERO at s = -0.5. Even though the poles are critically
    // damped, that zero produces a real (~14%) overshoot — a genuine property
    // of PD control of a double integrator, not a numerical artifact.
    Eigen::VectorXd num(1);
    num << 1.0;
    Eigen::VectorXd den(3);
    den << 1.0, 0.0, 0.0;  // s^2

    TransferFunction tf(num, den);
    ASSERT_EQ(tf.systemType(), 2);

    StateSpace plant = tf.toStateSpace();
    PIDController controller(1.0, 0.0, 2.0, 0.0);  // PD: Kp = 1, Kd = 2

    const int nSteps = 4000;
    Eigen::VectorXd time = linspace(0.0, 30.0, nSteps);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.order());
    const double setpoint = 1.0;

    SimulationResult result = simulateClosedLoopWithNewControl(
        plant, controller, setpoint, time, x0);

    StepMetrics metrics = StepResponseMetrics::compute(
        result.time, result.output, setpoint);

    EXPECT_NEAR(metrics.steadyStateValue, setpoint, 0.02)
        << "Type-2 plant with PD control should track the step. ss = "
        << metrics.steadyStateValue;
    EXPECT_LT(metrics.steadyStateError, 0.02)
        << "Type-2 plant has two integrators → ~zero steady-state error to a step.";
    // The PD zero at s = -0.5 adds overshoot, but the response stays well
    // damped and bounded — it does not ring or diverge.
    EXPECT_GT(metrics.percentOvershoot, 5.0)
        << "PD control of a double integrator overshoots due to its zero.";
    EXPECT_LT(metrics.percentOvershoot, 25.0)
        << "Overshoot should remain modest and bounded.";
}
