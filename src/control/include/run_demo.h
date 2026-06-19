#pragma once
#include <Eigen/Dense>
#include "transfer_function.h"
#include "closed_loop_simulator.h"
#include "step_response_metrics.h"

/**
 * @brief Result of the full demo pipeline.
 *
 * Bundles the closed-loop simulation output (time, output, control)
 * together with the computed step-response metrics (overshoot, settling
 * time, rise time, etc.) so that both are available to callers — whether
 * that caller is main.cc (which plots and prints them) or a headless
 * smoke test (which only checks them numerically).
 *
 * Factoring the numerical pipeline behind this return type is the key
 * design rule from Step 7.1: it keeps the "compute" path testable
 * without a GUI, so the demo can't silently rot.
 */
struct DemoResult {
    SimulationResult simulation;  /// Closed-loop trajectory (time, output, control)
    StepMetrics metrics;          /// Step-response quality numbers
};

/**
 * @brief Run the full TF → PID → simulate → metrics pipeline.
 *
 * This is the numerical core of the application, separated from
 * plotting so it can be tested headlessly. The pipeline:
 *
 *   1. Converts the transfer function to state-space (controllable
 *      canonical form via toStateSpace()).
 *   2. Builds a uniform time grid with linspace(0, tEnd, nSteps).
 *   3. Creates a PID controller with the given gains and offset.
 *   4. Runs the closed-loop simulation (unity feedback, ZOH
 *      integration via RK4).
 *   5. Computes step-response metrics (overshoot, rise time,
 *      settling time, steady-state error) from the output.
 *
 * @param plant      Transfer function G(s) of the plant to control.
 * @param kp         Proportional gain.
 * @param ki         Integral gain.
 * @param kd         Derivative gain.
 * @param setpoint   Desired output value (step reference).
 * @param tEnd       Simulation horizon in seconds.
 * @param nSteps     Number of time-grid points (default 10000).
 * @param offset     PID operating-point bias (default 0).
 * @return DemoResult containing the simulation trajectory and metrics.
 */
DemoResult runDemo(
    const TransferFunction& plant,
    double kp, double ki, double kd,
    double setpoint,
    double tEnd,
    int nSteps = 10000,
    double offset = 0.0);
