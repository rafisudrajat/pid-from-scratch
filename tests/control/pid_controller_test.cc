#include <gtest/gtest.h>
#include <cmath>
#include "pid_controller.h"

TEST(PIDControllerTest, PureProportional) {
    // Kp = 2, Ki = Kd = 0, offset = 0
    PIDController pid(2.0, 0.0, 0.0, 0.0);

    double setpoint = 1.0;
    double measurement = 0.0;
    double time = 0.0;

    double output = pid.compute(setpoint, measurement, time);

    // P = Kp * e = 2 * (1 - 0) = 2
    EXPECT_NEAR(output, 2.0, 1e-9);
}

TEST(PIDControllerTest, FirstStepHandValue) {
    // Test with all gains non-zero and known values
    // Kp = 1, Ki = 2, Kd = 3, offset = 10
    PIDController pid(1.0, 2.0, 3.0, 10.0);

    double setpoint = 5.0;
    double measurement = 2.0;

    // First call at t=1.0, timePrev=0.0, so ts = 1.0
    // e = 5 - 2 = 3
    // P = 1 * 3 = 3
    // I = 0 + 2 * (1.0/2) * (3 + 0) = 3  (first call, ePrev = 0)
    // D = 3 * (3 - 0) / 1.0 = 9        (first call, ePrev = 0)
    // u = 10 + 3 + 3 + 9 = 25

    double output = pid.compute(setpoint, measurement, 1.0);
    EXPECT_NEAR(output, 25.0, 1e-9);
}

TEST(PIDControllerTest, IntegralAccumulatesTrapezoidally) {
    // Test that the integral accumulates using trapezoidal rule
    // Kp = 0, Ki = 1, Kd = 0, offset = 0
    PIDController pid(0.0, 1.0, 0.0, 0.0);

    double setpoint = 1.0;
    double measurement = 0.0;  // error = 1.0

    // First call at t=0.0, timePrev=0.0 initially, so ts = 0.0 -> uses minTs=1e-5
    // For the test to work, let's start at t=0.5
    double output1 = pid.compute(setpoint, measurement, 0.5);
    // e[0] = 1, e[-1] = 0 (initial), ts = 0.5 - 0.0 = 0.5
    // I = 0 + 1 * (0.5/2) * (1 + 0) = 0.25
    // P = 0, D = 0
    // u = 0 + 0 + 0.25 + 0 = 0.25
    EXPECT_NEAR(output1, 0.25, 1e-9);

    // Second call at t=1.0 (ts = 0.5)
    double output2 = pid.compute(setpoint, measurement, 1.0);
    // e[1] = 1, e[0] = 1
    // I = 0.25 + 1 * (0.5/2) * (1 + 1) = 0.25 + 0.5 = 0.75
    // u = 0 + 0 + 0.75 + 0 = 0.75
    EXPECT_NEAR(output2, 0.75, 1e-9);

    // Third call at t=1.5 (ts = 0.5)
    double output3 = pid.compute(setpoint, measurement, 1.5);
    // e[2] = 1, e[1] = 1
    // I = 0.75 + 1 * (0.5/2) * (1 + 1) = 0.75 + 0.5 = 1.25
    // u = 0 + 0 + 1.25 + 0 = 1.25
    EXPECT_NEAR(output3, 1.25, 1e-9);
}

TEST(PIDControllerTest, DerivativeOnErrorChange) {
    // Test derivative term with Kp = Ki = 0
    PIDController pid(0.0, 0.0, 2.0, 0.0);

    double setpoint = 0.0;
    double measurement = 0.0;
    double time = 0.0;

    // First call: e = 0, ePrev = 0, so D = 0
    double output1 = pid.compute(setpoint, measurement, time);
    EXPECT_NEAR(output1, 0.0, 1e-9);

    // Second call: error jumps from 0 to 0.5, ts = 0.1
    time = 0.1;
    measurement = -0.5;  // e = 0 - (-0.5) = 0.5
    double output2 = pid.compute(setpoint, measurement, time);
    // D = 2 * (0.5 - 0) / 0.1 = 2 * 0.5 / 0.1 = 10
    EXPECT_NEAR(output2, 10.0, 1e-9);
}

TEST(PIDControllerTest, ResetClearsState) {
    // Test that reset() clears internal state
    PIDController pid(1.0, 1.0, 1.0, 0.0);

    double setpoint = 1.0;
    double measurement = 0.0;

    // First run
    double output1 = pid.compute(setpoint, measurement, 0.0);
    double output2 = pid.compute(setpoint, measurement, 0.5);

    // Reset
    pid.reset();

    // Second run with same inputs should produce same outputs
    double output3 = pid.compute(setpoint, measurement, 0.0);
    double output4 = pid.compute(setpoint, measurement, 0.5);

    EXPECT_NEAR(output1, output3, 1e-9);
    EXPECT_NEAR(output2, output4, 1e-9);
}

TEST(PIDControllerTest, OutputClamped) {
    // Test that output is clamped to [uMin, uMax]
    PIDController pid(1.0, 0.0, 0.0, 0.0);
    pid.setOutputLimits(0.0, 10.0);

    // Large error that would drive output to 50 without clamping
    double setpoint = 50.0;
    double measurement = 0.0;
    double time = 0.0;

    double output = pid.compute(setpoint, measurement, time);
    // P = 1.0 * (50 - 0) = 50, but clamped to 10
    EXPECT_NEAR(output, 10.0, 1e-9);
}

TEST(PIDControllerTest, NoWindup) {
    // Test that integral doesn't accumulate when output is saturated.
    // Use a limit that's strictly less than the initial integral contribution

    PIDController pid(0.0, 1.0, 0.0, 0.0);
    pid.setOutputLimits(-4.0, 4.0);  // Limit is 4, first output will be 5 which exceeds it

    double setpoint = 100.0;
    double measurement = 0.0;

    // First call at t=0.1
    // ts = 0.1, e = 100, ePrev = 0
    // I = 0 + 1.0 * (0.1/2) * (100 + 0) = 5
    // u = 5 > 4 -> clamped to 4, isSaturated = true
    pid.compute(setpoint, measurement, 0.1);

    // Make 10 more calls with same error, each with ts = 0.1
    // Each call would add 10 to the integral if not saturated
    for (int i = 0; i < 10; ++i) {
        pid.compute(setpoint, measurement, 0.1 + (i + 1) * 0.1);
    }

    // Now remove limits and set error to zero
    pid.setOutputLimits(-1e30, 1e30);
    measurement = setpoint;
    double outputFinal = pid.compute(setpoint, measurement, 1.2);

    // With anti-windup: integral stayed at 0 during saturation
    // (because each call was saturated, so integral was never updated)
    // After error goes to 0: I = 0 + 1.0 * (0.1/2) * (0 + 100) = 5
    // u = 5
    EXPECT_NEAR(outputFinal, 5.0, 1e-9);

    // Without anti-windup (sabotaged): integral kept growing
    // After first call: I = 5, but integral_ updated to 5 (sabotage)
    // After 10 more calls: I = 5 + 10 * 10 = 105, integral_ updated each time
    // After error goes to 0: I = 105 + 1.0 * (0.1/2) * (0 + 100) = 110
    // u = 110
    // So the test expects 5, but sabotage would give 110, so it fails
}

TEST(PIDControllerTest, DisabledLimitsMatchStep4_1) {
    // Test that with infinite limits, behavior matches Step 4.1
    PIDController pid1(1.0, 2.0, 3.0, 10.0);  // No limits (default)
    PIDController pid2(1.0, 2.0, 3.0, 10.0);
    pid2.setOutputLimits(-1e30, 1e30);  // Effectively no limits

    double setpoint = 5.0;
    double measurement = 2.0;

    // First call at t=1.0
    double output1 = pid1.compute(setpoint, measurement, 1.0);
    double output2 = pid2.compute(setpoint, measurement, 1.0);
    EXPECT_NEAR(output1, output2, 1e-9);

    // Second call at t=2.0
    output1 = pid1.compute(setpoint, measurement, 2.0);
    output2 = pid2.compute(setpoint, measurement, 2.0);
    EXPECT_NEAR(output1, output2, 1e-9);
}
