#pragma once

/**
 * @brief Discretized PID controller with anti-windup.
 *
 * Implements a stateful PID controller that computes control output
 * using discretized proportional, integral (trapezoidal rule), and
 * derivative (backward difference) terms, with optional output clamping
 * and anti-windup protection.
 *
 * The controller maintains internal state: integral accumulator,
 * previous error, previous time between calls, and output limits.
 */
class PIDController {
public:
    /**
     * @brief Construct a PID controller with given gains.
     *
     * @param kp        Proportional gain.
     * @param ki        Integral gain.
     * @param kd        Derivative gain.
     * @param offset    Operating point bias (default 0).
     */
    PIDController(double kp, double ki, double kd, double offset = 0.0);

    /**
     * @brief Set output limits for clamping and anti-windup.
     *
     * When the output is clamped to these limits, the integral term
     * stops accumulating (anti-windup).
     *
     * @param min Output minimum (default -infinity, no limit).
     * @param max Output maximum (default +infinity, no limit).
     */
    void setOutputLimits(double min = -1e30, double max = 1e30);

    /**
     * @brief Compute the control output.
     *
     * Given the current setpoint and measurement, computes the PID output
     * using:
     *   P = Kp * e[k]
     *   I[k] = I[k-1] + Ki * (T_s / 2) * (e[k] + e[k-1])  (trapezoidal rule)
     *   D = Kd * (e[k] - e[k-1]) / T_s                  (backward difference)
     *   u[k] = offset + P + I[k] + D
     *
     * Where T_s = time - time_prev is the elapsed time since the last call.
     *
     * If output limits are set and u[k] exceeds them, u[k] is clamped and
     * the integral accumulator is NOT updated (anti-windup).
     *
     * On the first call, e_prev is treated as 0, and time_prev is set to the
     * first time value. If T_s <= 0, a small T_s = 1e-5 is used to avoid
     * division by zero.
     *
     * @param setpoint     Desired value.
     * @param measurement   Current measured value.
     * @param time         Current time.
     * @return Control output u (clamped if limits are set).
     */
    double compute(double setpoint, double measurement, double time);

    /**
     * @brief Reset the controller's internal state.
     *
     * Clears the integral accumulator, previous error, and previous time.
     */
    void reset();

private:
    double kp_;     // Proportional gain
    double ki_;     // Integral gain
    double kd_;     // Derivative gain
    double offset_; // Operating point bias
    
    // Internal state
    double integral_;   // Integral accumulator
    double e_prev_;     // Previous error
    double time_prev_; // Previous time
    bool first_call_;  // True if this is the first call
    
    // Output limits
    double u_min_;     // Minimum output (default -infinity)
    double u_max_;     // Maximum output (default +infinity)
    bool has_limits_;  // True if limits are set
};
