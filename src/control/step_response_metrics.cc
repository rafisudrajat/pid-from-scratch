#include "step_response_metrics.h"
#include <cmath>
#include <algorithm>

StepMetrics StepResponseMetrics::compute(
    const Eigen::VectorXd& time,
    const Eigen::VectorXd& output,
    double setpoint) {

    StepMetrics metrics;

    // Validate inputs
    if (time.size() == 0 || output.size() == 0) {
        // Return default-initialized metrics
        return metrics;
    }
    if (time.size() != output.size()) {
        throw std::invalid_argument("time and output must have the same length");
    }

    // Compute steady-state value as the average of the tail (last 10% of points)
    // This handles cases where there might be small oscillations at the end
    int tailStart = static_cast<int>(output.size() * 0.9);
    if (tailStart < 0) tailStart = 0;

    double ssSum = 0.0;
    for (int i = tailStart; i < output.size(); ++i) {
        ssSum += output(i);
    }
    metrics.steadyStateValue = ssSum / (output.size() - tailStart);
    metrics.steadyStateError = std::abs(setpoint - metrics.steadyStateValue);

    // Find peak value and peak time
    double peakValue = output(0);
    double peakTime = time(0);
    bool hasOvershoot = false;

    for (int i = 1; i < output.size(); ++i) {
        if (output(i) > peakValue) {
            peakValue = output(i);
            peakTime = time(i);
        }
    }

    // Percent overshoot: max(0, (peakValue - ssValue) / ssValue) * 100
    // Only count as overshoot if peak > ssValue
    if (peakValue > metrics.steadyStateValue && metrics.steadyStateValue > 0) {
        metrics.percentOvershoot = ((peakValue - metrics.steadyStateValue) / metrics.steadyStateValue) * 100.0;
        hasOvershoot = true;
    } else {
        metrics.percentOvershoot = 0.0;
        hasOvershoot = false;
    }

    metrics.peakTime = hasOvershoot ? peakTime : 0.0;

    // Compute rise time (time from 10% to 90% of steady-state value)
    double tenPercent = metrics.steadyStateValue * 0.1;
    double ninetyPercent = metrics.steadyStateValue * 0.9;

    int riseStartIdx = -1;
    int riseEndIdx = -1;

    // Find first crossing of 10%
    for (int i = 0; i < output.size(); ++i) {
        if (output(i) >= tenPercent) {
            riseStartIdx = i;
            break;
        }
    }

    // Find first crossing of 90%
    for (int i = (riseStartIdx >= 0 ? riseStartIdx : 0); i < output.size(); ++i) {
        if (output(i) >= ninetyPercent) {
            riseEndIdx = i;
            break;
        }
    }

    if (riseStartIdx >= 0 && riseEndIdx >= 0 && riseEndIdx > riseStartIdx) {
        // Linear interpolation for more accuracy
        // For 10% crossing
        int idx10 = riseStartIdx;
        if (idx10 > 0 && output(idx10 - 1) < tenPercent && output(idx10) > tenPercent) {
            double frac = (tenPercent - output(idx10 - 1)) / (output(idx10) - output(idx10 - 1));
            metrics.riseTime = time(idx10 - 1) + frac * (time(idx10) - time(idx10 - 1));
        } else {
            metrics.riseTime = time(idx10);
        }

        // For 90% crossing
        int idx90 = riseEndIdx;
        double time90;
        if (idx90 > 0 && output(idx90 - 1) < ninetyPercent && output(idx90) > ninetyPercent) {
            double frac = (ninetyPercent - output(idx90 - 1)) / (output(idx90) - output(idx90 - 1));
            time90 = time(idx90 - 1) + frac * (time(idx90) - time(idx90 - 1));
        } else {
            time90 = time(idx90);
        }

        metrics.riseTime = time90 - metrics.riseTime;
    } else {
        metrics.riseTime = 0.0;
    }

    // Compute settling time (2% band)
    // The settling band is [0.98 * ssValue, 1.02 * ssValue]
    // We need the last time the output exits this band
    double lowerBand = metrics.steadyStateValue * 0.98;
    double upperBand = metrics.steadyStateValue * 1.02;

    // Work backwards from the end to find the last time outside the band
    int settlingIdx = static_cast<int>(output.size()) - 1;
    for (int i = static_cast<int>(output.size()) - 1; i >= 0; --i) {
        if (output(i) < lowerBand || output(i) > upperBand) {
            settlingIdx = i;
            break;
        }
    }

    // If all points are within the band, settling time is 0
    // Otherwise, it's the time at settlingIdx
    // But we need to ensure we've actually settled, so look for the point
    // where it STAYS within the band

    // More robust approach: find the last time it ENTERS and STAYS in the band
    bool inBand = false;
    int lastExitIdx = -1;
    for (int i = 0; i < output.size(); ++i) {
        bool currentInBand = (output(i) >= lowerBand && output(i) <= upperBand);

        if (!currentInBand) {
            lastExitIdx = i;
        }
    }

    if (lastExitIdx >= 0 && lastExitIdx < static_cast<int>(output.size()) - 1) {
        // Linear interpolation for the exact crossing time
        int idx = lastExitIdx;
        if (output(idx) > upperBand) {
            // Crossing from above
            if (idx + 1 < output.size() && output(idx + 1) <= upperBand) {
                double frac = (upperBand - output(idx)) / (output(idx + 1) - output(idx));
                metrics.settlingTime = time(idx) + frac * (time(idx + 1) - time(idx));
            } else {
                metrics.settlingTime = time(idx);
            }
        } else if (output(idx) < lowerBand) {
            // Crossing from below
            if (idx + 1 < output.size() && output(idx + 1) >= lowerBand) {
                double frac = (lowerBand - output(idx)) / (output(idx + 1) - output(idx));
                metrics.settlingTime = time(idx) + frac * (time(idx + 1) - time(idx));
            } else {
                metrics.settlingTime = time(idx);
            }
        } else {
            metrics.settlingTime = time(idx);
        }
    } else {
        metrics.settlingTime = 0.0;
    }

    return metrics;
}
