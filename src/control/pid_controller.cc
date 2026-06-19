#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double offset)
    : kp_(kp), ki_(ki), kd_(kd), offset_(offset),
      integral_(0.0), e_prev_(0.0), time_prev_(0.0), first_call_(true),
      u_min_(-1e30), u_max_(1e30), has_limits_(false)
{
}

void PIDController::setOutputLimits(double min, double max) {
    u_min_ = min;
    u_max_ = max;
    has_limits_ = true;
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
    
    // Derivative term
    double D = 0.0;
    if (first_call_) {
        D = kd_ * (e - 0.0) / T_s;
    } else {
        D = kd_ * (e - e_prev_) / T_s;
    }
    
    // Integral term with anti-windup
    double I = integral_;
    if (first_call_) {
        I += ki_ * (T_s / 2.0) * (e + 0.0);
    } else {
        I += ki_ * (T_s / 2.0) * (e + e_prev_);
    }
    
    // Compute raw (unclamped) output
    double u_raw = offset_ + P + I + D;
    
    // Check for saturation and apply anti-windup
    bool is_saturated = false;
    double u = u_raw;
    
    if (has_limits_) {
        if (u_raw < u_min_) {
            u = u_min_;
            is_saturated = true;
        } else if (u_raw > u_max_) {
            u = u_max_;
            is_saturated = true;
        }
    }
    
    // Update state for next call
    // Anti-windup: only update integral if NOT saturated
    if (!is_saturated) {
        integral_ = I;
    }
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
