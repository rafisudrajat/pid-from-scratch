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
    int tail_start = static_cast<int>(output.size() * 0.9);
    if (tail_start < 0) tail_start = 0;
    
    double ss_sum = 0.0;
    for (int i = tail_start; i < output.size(); ++i) {
        ss_sum += output(i);
    }
    metrics.steadyStateValue = ss_sum / (output.size() - tail_start);
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
    double ten_percent = metrics.steadyStateValue * 0.1;
    double ninety_percent = metrics.steadyStateValue * 0.9;
    
    int rise_start_idx = -1;
    int rise_end_idx = -1;
    
    // Find first crossing of 10%
    for (int i = 0; i < output.size(); ++i) {
        if (output(i) >= ten_percent) {
            rise_start_idx = i;
            break;
        }
    }
    
    // Find first crossing of 90%
    for (int i = (rise_start_idx >= 0 ? rise_start_idx : 0); i < output.size(); ++i) {
        if (output(i) >= ninety_percent) {
            rise_end_idx = i;
            break;
        }
    }
    
    if (rise_start_idx >= 0 && rise_end_idx >= 0 && rise_end_idx > rise_start_idx) {
        // Linear interpolation for more accuracy
        // For 10% crossing
        int idx10 = rise_start_idx;
        if (idx10 > 0 && output(idx10 - 1) < ten_percent && output(idx10) > ten_percent) {
            double frac = (ten_percent - output(idx10 - 1)) / (output(idx10) - output(idx10 - 1));
            metrics.riseTime = time(idx10 - 1) + frac * (time(idx10) - time(idx10 - 1));
        } else {
            metrics.riseTime = time(idx10);
        }
        
        // For 90% crossing
        int idx90 = rise_end_idx;
        double time90;
        if (idx90 > 0 && output(idx90 - 1) < ninety_percent && output(idx90) > ninety_percent) {
            double frac = (ninety_percent - output(idx90 - 1)) / (output(idx90) - output(idx90 - 1));
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
    double lower_band = metrics.steadyStateValue * 0.98;
    double upper_band = metrics.steadyStateValue * 1.02;
    
    // Work backwards from the end to find the last time outside the band
    int settling_idx = static_cast<int>(output.size()) - 1;
    for (int i = static_cast<int>(output.size()) - 1; i >= 0; --i) {
        if (output(i) < lower_band || output(i) > upper_band) {
            settling_idx = i;
            break;
        }
    }
    
    // If all points are within the band, settling time is 0
    // Otherwise, it's the time at settling_idx
    // But we need to ensure we've actually settled, so look for the point
    // where it STAYS within the band
    
    // More robust approach: find the last time it ENTERS and STAYS in the band
    bool in_band = false;
    int last_exit_idx = -1;
    for (int i = 0; i < output.size(); ++i) {
        bool current_in_band = (output(i) >= lower_band && output(i) <= upper_band);
        
        if (!current_in_band) {
            last_exit_idx = i;
        }
    }
    
    if (last_exit_idx >= 0 && last_exit_idx < static_cast<int>(output.size()) - 1) {
        // Linear interpolation for the exact crossing time
        int idx = last_exit_idx;
        if (output(idx) > upper_band) {
            // Crossing from above
            if (idx + 1 < output.size() && output(idx + 1) <= upper_band) {
                double frac = (upper_band - output(idx)) / (output(idx + 1) - output(idx));
                metrics.settlingTime = time(idx) + frac * (time(idx + 1) - time(idx));
            } else {
                metrics.settlingTime = time(idx);
            }
        } else if (output(idx) < lower_band) {
            // Crossing from below
            if (idx + 1 < output.size() && output(idx + 1) >= lower_band) {
                double frac = (lower_band - output(idx)) / (output(idx + 1) - output(idx));
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
