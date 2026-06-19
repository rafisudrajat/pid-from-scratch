#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double offset)
    : kp_(kp), ki_(ki), kd_(kd), offset_(offset),
      integral_(0.0), ePrev_(0.0), timePrev_(0.0), firstCall_(true),
      uMin_(-1e30), uMax_(1e30), hasLimits_(false)
{
}

void PIDController::setOutputLimits(double min, double max) {
    uMin_ = min;
    uMax_ = max;
    hasLimits_ = true;
}

double PIDController::compute(double setpoint, double measurement, double time) {
    // Calculate current error
    double e = setpoint - measurement;

    // Calculate time step
    double ts = time - timePrev_;

    // Guard against non-positive time step
    const double minTs = 1e-5;
    if (ts <= 0.0) {
        ts = minTs;
    }

    // Proportional term
    double P = kp_ * e;

    // Derivative term
    double D = 0.0;
    if (firstCall_) {
        D = kd_ * (e - 0.0) / ts;
    } else {
        D = kd_ * (e - ePrev_) / ts;
    }

    // Integral term with anti-windup
    double I = integral_;
    if (firstCall_) {
        I += ki_ * (ts / 2.0) * (e + 0.0);
    } else {
        I += ki_ * (ts / 2.0) * (e + ePrev_);
    }

    // Compute raw (unclamped) output
    double uRaw = offset_ + P + I + D;

    // Check for saturation and apply anti-windup
    bool isSaturated = false;
    double u = uRaw;

    if (hasLimits_) {
        if (uRaw < uMin_) {
            u = uMin_;
            isSaturated = true;
        } else if (uRaw > uMax_) {
            u = uMax_;
            isSaturated = true;
        }
    }

    // Update state for next call
    // Anti-windup: only update integral if NOT saturated
    if (!isSaturated) {
        integral_ = I;
    }
    ePrev_ = e;
    timePrev_ = time;
    firstCall_ = false;

    return u;
}

void PIDController::reset() {
    integral_ = 0.0;
    ePrev_ = 0.0;
    timePrev_ = 0.0;
    firstCall_ = true;
}

void PIDController::setInitialTime(double timePrev) {
    timePrev_ = timePrev;
}
