#include "run_demo.h"
#include "state_space.h"
#include "pid_controller.h"
#include "linspace.h"

DemoResult runDemo(
    const TransferFunction& plant,
    double kp, double ki, double kd,
    double setpoint,
    double tEnd,
    int nSteps,
    double offset) {

    // Step 1: Convert transfer function to state-space realization
    // (controllable canonical form — companion A, poles = eig(A))
    StateSpace ss = plant.toStateSpace();

    // Step 2: Build the time grid from 0 to tEnd
    Eigen::VectorXd time = linspace(0.0, tEnd, nSteps);

    // Step 3: Create the PID controller with given gains and offset
    PIDController controller(kp, ki, kd, offset);

    // Step 4: Initial state — all zeros (system starts at rest)
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(ss.order());

    // Step 5: Run the closed-loop simulation (unity feedback, ZOH)
    SimulationResult simResult = ClosedLoopSimulator::simulate(
        ss, controller, setpoint, time, x0);

    // Step 6: Compute step-response metrics from the output
    StepMetrics metrics = StepResponseMetrics::compute(
        simResult.time, simResult.output, setpoint);

    // Bundle and return
    DemoResult result;
    result.simulation = simResult;
    result.metrics = metrics;
    return result;
}
