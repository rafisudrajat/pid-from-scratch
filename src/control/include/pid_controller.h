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
     *   I[k] = I[k-1] + Ki * (Ts / 2) * (e[k] + e[k-1])  (trapezoidal rule)
     *   D = Kd * (e[k] - e[k-1]) / Ts                  (backward difference)
     *   u[k] = offset + P + I[k] + D
     *
     * Where Ts = time - timePrev is the elapsed time since the last call.
     *
     * If output limits are set and u[k] exceeds them, u[k] is clamped and
     * the integral accumulator is NOT updated (anti-windup).
     *
     * On the first call, ePrev is treated as 0, and timePrev is set to the
     * first time value. If Ts <= 0, a small Ts = 1e-5 is used to avoid
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
    double ePrev_;      // Previous error
    double timePrev_;   // Previous time
    bool firstCall_;    // True if this is the first call

    // Output limits
    double uMin_;      // Minimum output (default -infinity)
    double uMax_;      // Maximum output (default +infinity)
    bool hasLimits_;   // True if limits are set
};
