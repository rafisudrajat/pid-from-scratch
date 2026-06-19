#pragma once
#include <Eigen/Dense>
#include "state_space.h"
#include "pid_controller.h"

/**
 * @brief Struct to hold simulation results.
 */
struct SimulationResult {
    Eigen::VectorXd time;      /// Time points
    Eigen::VectorXd output;    /// System output at each time point
    Eigen::VectorXd control;   /// Control input at each time point
};

/**
 * @brief Closed-loop PID simulator.
 *
 * Simulates a unity-feedback control loop: setpoint -> error -> PID -> plant -> measurement -> feedback.
 * At each time step:
 *   1. Measure y[k] = C * x[k] + D * u[k-1] (with u[-1] = 0)
 *   2. Compute u[k] = controller.compute(setpoint, y[k], t[k])
 *   3. Integrate plant with u[k] held constant (ZOH) using RK4
 *   4. Record y[k] and u[k]
 */
class ClosedLoopSimulator {
public:
    /**
     * @brief Run a closed-loop simulation.
     *
     * @param plant       State-space model of the plant.
     * @param controller  PID controller.
     * @param setpoint    Desired output value.
     * @param time        Time grid (vector of time points).
     * @param x0          Initial state vector.
     * @return SimulationResult containing time, output, and control vectors.
     */
    static SimulationResult simulate(
        const StateSpace& plant,
        PIDController& controller,
        double setpoint,
        const Eigen::VectorXd& time,
        const Eigen::VectorXd& x0);

private:
    ClosedLoopSimulator() = delete;  // Prevent instantiation - all static
};
