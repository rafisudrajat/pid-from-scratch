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

    // First call at t=1.0, time_prev=0.0, so T_s = 1.0
    // e = 5 - 2 = 3
    // P = 1 * 3 = 3
    // I = 0 + 2 * (1.0/2) * (3 + 0) = 3  (first call, e_prev = 0)
    // D = 3 * (3 - 0) / 1.0 = 9        (first call, e_prev = 0)
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
    
    // First call at t=0.0, time_prev=0.0 initially, so T_s = 0.0 -> uses min_Ts=1e-5
    // For the test to work, let's start at t=0.5
    double output1 = pid.compute(setpoint, measurement, 0.5);
    // e[0] = 1, e[-1] = 0 (initial), T_s = 0.5 - 0.0 = 0.5
    // I = 0 + 1 * (0.5/2) * (1 + 0) = 0.25
    // P = 0, D = 0
    // u = 0 + 0 + 0.25 + 0 = 0.25
    EXPECT_NEAR(output1, 0.25, 1e-9);

    // Second call at t=1.0 (T_s = 0.5)
    double output2 = pid.compute(setpoint, measurement, 1.0);
    // e[1] = 1, e[0] = 1
    // I = 0.25 + 1 * (0.5/2) * (1 + 1) = 0.25 + 0.5 = 0.75
    // u = 0 + 0 + 0.75 + 0 = 0.75
    EXPECT_NEAR(output2, 0.75, 1e-9);

    // Third call at t=1.5 (T_s = 0.5)
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

    // First call: e = 0, e_prev = 0, so D = 0
    double output1 = pid.compute(setpoint, measurement, time);
    EXPECT_NEAR(output1, 0.0, 1e-9);

    // Second call: error jumps from 0 to 0.5, T_s = 0.1
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
