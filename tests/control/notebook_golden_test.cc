#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <vector>
#include "state_space.h"
#include "pid_controller.h"
#include "closed_loop_simulator.h"
#include "ode.h"

// Fallback so IntelliSense doesn't flag the CMake-injected macro as undefined
#ifndef TEST_DATA_DIR
#define TEST_DATA_DIR "tests/control/data"
#endif

TEST(NotebookGoldenTest, MatchesNotebookTrajectory) {
    // Step 8.1: Reproduce the notebook's thermal-plant PID run
    // Notebook parameters:
    // - Plant: dT/dt = 1/(τ(1+ε)) * (T_f - T) + Q/(1+ε) * (T_q - T)
    //   with ε = 1, τ = 4, T_f = 300, Q = 2
    //   => dT/dt = 37.5 + T_q - 1.125 * T
    // - PID: Kp = 0.6, Ki = 0.2, Kd = 0.1, offset = 320
    // - Setpoint = 310, Initial T = 300
    // - Time grid: 300 steps, deltat = 0.1 s, total time = 30 s
    //
    // StateSpace realization with augmented state:
    //   x1 = T - 37.5/1.125, x2 = 1 (constant state for offset)
    //   A = [[-1.125, 0], [0, 0]], B = [[1], [0]]
    //   C = [[1, 37.5/1.125]], D = [0]
    //   x0 = [300 - 37.5/1.125, 1]
    //
    // Notebook structure: computes control BEFORE integrating (different from standard ZOH
    // which uses previous control value), and uses 10 substeps per integration interval.

    const double constantOffset = 37.5 / 1.125;
    
    Eigen::MatrixXd A(2, 2);
    A << -1.125, 0.0,
         0.0,   0.0;
    
    Eigen::MatrixXd B(2, 1);
    B << 1.0, 0.0;
    
    Eigen::MatrixXd C(1, 2);
    C << 1.0, constantOffset;
    
    Eigen::MatrixXd D(1, 1);
    D << 0.0;
    
    StateSpace plant(A, B, C, D);

    // PID controller with notebook parameters
    // Match notebook's initialization: global time_prev = -1e-6
    PIDController controller(0.6, 0.2, 0.1, 320.0);
    controller.setInitialTime(-1e-6);

    // Time grid: 300 steps from 0 to 29.9 (deltat = 0.1)
    const int nSteps = 300;
    const double deltat = 0.1;
    Eigen::VectorXd time_vec(nSteps);
    for (int i = 0; i < nSteps; ++i) {
        time_vec(i) = i * deltat;
    }

    // Initial state: x1 = T(0) - constantOffset, x2 = 1
    Eigen::VectorXd x0(2);
    x0 << (300.0 - constantOffset), 1.0;

    // Setpoint
    const double setpoint = 310.0;

    // Manual simulation loop matching the notebook's structure:
    // Computes control BEFORE integrating, using new control value for integration.
    SimulationResult result;
    result.time = time_vec;
    result.output = Eigen::VectorXd::Zero(nSteps);
    result.control = Eigen::VectorXd::Zero(nSteps);
    
    Eigen::VectorXd x = x0;
    ODESolver solver;
    
    // Set initial control value to offset (matching notebook's q_sol[0] = 320)
    result.control(0) = 320.0;
    
    // First output measurement at t=0
    result.output(0) = plant.output(x, 320.0);
    
    // Run the loop from k=1 to nSteps-1
    for (int k = 1; k < nSteps; ++k) {
        double current_time = time_vec(k);
        double prev_time = time_vec(k - 1);
        double stepSize = current_time - prev_time;
        
        // Compute u[k] using previous output y[k-1]
        result.control(k) = controller.compute(setpoint, result.output(k - 1), current_time);
        double uCurrent = result.control(k);
        
        // Integrate with 10 substeps (matching notebook: tspan = np.linspace(time_prev, time, 10))
        const int nSubsteps = 9;  // 10 points = 9 substeps
        double subStepSize = stepSize / nSubsteps;
        
        auto f = [&plant, uCurrent](double t, const Eigen::VectorXd& xState) -> Eigen::VectorXd {
            return plant.derivative(xState, uCurrent);
        };
        
        // Integrate with nSubsteps
        for (int s = 0; s < nSubsteps; ++s) {
            x = solver.rk4Step(f, prev_time + s * subStepSize, x, subStepSize);
        }
        
        // Measure y[k] = C * x[k] + D * u[k]
        result.output(k) = plant.output(x, uCurrent);
    }

    // Load reference data from CSV file (path injected by CMake)
    std::ifstream refFile(std::string(TEST_DATA_DIR) + "/notebook_reference.csv");
    ASSERT_TRUE(refFile.is_open()) << "Could not open reference data file";

    // Skip header line
    std::string header;
    std::getline(refFile, header);

    // Read reference data
    std::vector<double> refTime, refOutput, refControl;
    std::string line;
    while (std::getline(refFile, line)) {
        std::istringstream iss(line);
        std::string token;
        double t, y, u;
        
        std::getline(iss, token, ',');
        t = std::stod(token);
        std::getline(iss, token, ',');
        y = std::stod(token);
        std::getline(iss, token, ',');
        u = std::stod(token);
        
        refTime.push_back(t);
        refOutput.push_back(y);
        refControl.push_back(u);
    }

    // Check that we have the same number of points
    EXPECT_EQ(result.time.size(), refTime.size());
    EXPECT_EQ(result.output.size(), refOutput.size());
    EXPECT_EQ(result.control.size(), refControl.size());

    // Compare trajectories (tolerance 1e-2 as specified in the roadmap)
    const double tolerance = 1e-2;
    
    for (int i = 0; i < result.time.size(); ++i) {
        // Check time points match (they should be identical)
        EXPECT_NEAR(result.time(i), refTime[i], tolerance) << "Time mismatch at index " << i;
        
        // Check output (temperature) matches
        EXPECT_NEAR(result.output(i), refOutput[i], tolerance)
            << "Output mismatch at t=" << result.time(i)
            << ": C++=" << result.output(i) << ", notebook=" << refOutput[i];
        
        // Check control (T_q) matches
        EXPECT_NEAR(result.control(i), refControl[i], tolerance)
            << "Control mismatch at t=" << result.time(i)
            << ": C++=" << result.control(i) << ", notebook=" << refControl[i];
    }
}
