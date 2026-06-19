#include "closed_loop_simulator.h"
#include "ode.h"

SimulationResult ClosedLoopSimulator::simulate(
    const StateSpace& plant,
    PIDController& controller,
    double setpoint,
    const Eigen::VectorXd& time,
    const Eigen::VectorXd& x0) {

    int nSteps = static_cast<int>(time.size());

    // Initialize result vectors
    SimulationResult result;
    result.time = time;
    result.output = Eigen::VectorXd::Zero(nSteps);
    result.control = Eigen::VectorXd::Zero(nSteps);

    // Current state
    Eigen::VectorXd x = x0;

    // ODE solver
    ODESolver solver;

    // Previous control input (u[k-1])
    double u_prev = 0.0;

    // Reset controller to ensure clean state
    controller.reset();

    // First step: k = 0
    // Measure y[0] = C * x[0] + D * u[-1] (u[-1] = 0)
    result.output(0) = plant.output(x, u_prev);
    
    // Compute u[0] = controller.compute(setpoint, y[0], t[0])
    result.control(0) = controller.compute(setpoint, result.output(0), time(0));
    
    // For the first step, we don't integrate yet - we need at least 2 points
    // But we do need to set u_prev for the next measurement
    u_prev = result.control(0);

    // For k = 1 to nSteps-1:
    for (int k = 1; k < nSteps; ++k) {
        double stepSize = time(k) - time(k - 1);

        // Step 1: Integrate the plant forward from x[k-1] to x[k] with u[k-1] constant (ZOH)
        // Create the ODE function: dx/dt = A * x + B * u, where u = u_prev is constant
        auto f = [&plant, u_prev](double t, const Eigen::VectorXd& x_state) -> Eigen::VectorXd {
            return plant.derivative(x_state, u_prev);
        };

        x = solver.rk4Step(f, time(k - 1), x, stepSize);

        // Step 2: Measure y[k] = C * x[k] + D * u[k-1]
        result.output(k) = plant.output(x, u_prev);

        // Step 3: Compute u[k] = controller.compute(setpoint, y[k], t[k])
        result.control(k) = controller.compute(setpoint, result.output(k), time(k));

        // Update u_prev for next iteration
        u_prev = result.control(k);
    }

    return result;
}
