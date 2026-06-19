#pragma once
#include <Eigen/Dense>

/**
 * @brief Struct to hold step response metrics.
 */
struct StepMetrics {
    double steadyStateValue;    /// Final settled value of the output
    double steadyStateError;    /// |setpoint - steadyStateValue|
    double percentOvershoot;     /// Percentage overshoot (0-100), 0 if no overshoot
    double peakTime;             /// Time at which the peak occurs (0 if no overshoot)
    double riseTime;             /// Time to go from 10% to 90% of steady-state value
    double settlingTime;         /// Time at which output last exits the ±2% band
};

/**
 * @brief Utilities for computing step response metrics from time series data.
 *
 * Computes metrics from a step response (time, output) pair:
 * - Steady-state value (average of tail)
 * - Steady-state error
 * - Percent overshoot
 * - Peak time
 * - Rise time (10% to 90%)
 * - Settling time (2% band)
 */
class StepResponseMetrics {
public:
    /**
     * @brief Compute step response metrics from time and output data.
     *
     * @param time      Time points (must be monotonically increasing).
     * @param output    Output values at each time point.
     * @param setpoint  Desired steady-state value.
     * @return StepMetrics containing all computed metrics.
     */
    static StepMetrics compute(
        const Eigen::VectorXd& time,
        const Eigen::VectorXd& output,
        double setpoint);

private:
    StepResponseMetrics() = delete;  // Prevent instantiation - all static
};
