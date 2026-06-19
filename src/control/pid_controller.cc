#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double offset)
    : kp_(kp), ki_(ki), kd_(kd), offset_(offset),
      integral_(0.0), e_prev_(0.0), time_prev_(0.0), first_call_(true)
{
}

double PIDController::compute(double setpoint, double measurement, double time) {
    // Calculate current error
    double e = setpoint - measurement;
    
    // Calculate time step
    double T_s = time - time_prev_;
    
    // Guard against non-positive time step
    const double min_Ts = 1e-5;
    if (T_s <= 0.0) {
        T_s = min_Ts;
    }
    
    // Proportional term
    double P = kp_ * e;
    
    // Integral term (trapezoidal rule)
    double I = integral_;
    if (first_call_) {
        // On first call, treat e_prev as 0
        I += ki_ * (T_s / 2.0) * (e + 0.0);
    } else {
        I += ki_ * (T_s / 2.0) * (e + e_prev_);
    }
    
    // Derivative term (backward difference)
    double D = 0.0;
    if (first_call_) {
        // On first call, treat e_prev as 0
        D = kd_ * (e - 0.0) / T_s;
    } else {
        D = kd_ * (e - e_prev_) / T_s;
    }
    
    // Compute output
    double u = offset_ + P + I + D;
    
    // Update state for next call
    integral_ = I;
    e_prev_ = e;
    time_prev_ = time;
    first_call_ = false;
    
    return u;
}

void PIDController::reset() {
    integral_ = 0.0;
    e_prev_ = 0.0;
    time_prev_ = 0.0;
    first_call_ = true;
}
