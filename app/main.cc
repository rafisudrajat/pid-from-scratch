#include <iostream>
#include <vector>
#include <iomanip>
#include "matplotlibcpp.h"
#include "run_demo.h"
#include "transfer_function.h"

namespace plt = matplotlibcpp;

/**
 * @brief Convert an Eigen::VectorXd to a std::vector<double>.
 *
 * matplotlib-cpp requires std::vector for plotting, while the
 * simulation pipeline returns Eigen::VectorXd. This adaptor
 * bridges the two representations.
 */
std::vector<double> toStdVector(const Eigen::VectorXd& v) {
    return std::vector<double>(v.data(), v.data() + v.size());
}

int main()
{
    plt::initialize_python();

    // ── Define the plant ─────────────────────────────────────────────
    // First-order plant: G(s) = 2 / (s + 1)
    //   DC gain K = 2     — a unit step settles at 2 in open loop
    //   Time constant τ = 1 s — reaches 63.2% of final value after 1 s
    // This is a stable, strictly proper system — the simplest non-trivial
    // demo that exercises the full TF → PID → plot pipeline.
    Eigen::VectorXd num(1);
    num << 2.0;                  // numerator coefficients (descending powers)
    Eigen::VectorXd den(2);
    den << 1.0, 1.0;            // denominator: s + 1

    TransferFunction plant(num, den);

    // ── PID gains and simulation parameters ──────────────────────────
    // PI controller (no derivative to avoid the initial kick):
    //   Kp = 2   — proportional gain drives fast response
    //   Ki = 1   — integral action eliminates steady-state error
    //   Kd = 0   — no derivative (clean first demo; add later for damping)
    double kp = 2.0;
    double ki = 1.0;
    double kd = 0.0;
    double setpoint = 1.0;      // desired output value
    double tEnd = 10.0;         // simulation horizon (seconds)
    int nSteps = 5000;          // time-grid resolution

    // ── Run the numerical pipeline ───────────────────────────────────
    // runDemo() exercises the entire stack:
    //   linspace → toStateSpace → PIDController → ClosedLoopSimulator
    //   → StepResponseMetrics
    // It returns the trajectory and metrics with no GUI dependency.
    DemoResult result = runDemo(plant, kp, ki, kd, setpoint, tEnd, nSteps);

    // ── Print step-response metrics to the console ───────────────────
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n=== Step-Response Metrics ===" << std::endl;
    std::cout << "  Steady-state value : " << result.metrics.steadyStateValue << std::endl;
    std::cout << "  Steady-state error : " << result.metrics.steadyStateError << std::endl;
    std::cout << "  Percent overshoot  : " << result.metrics.percentOvershoot << " %" << std::endl;
    std::cout << "  Rise time          : " << result.metrics.riseTime          << " s" << std::endl;
    std::cout << "  Settling time (2%) : " << result.metrics.settlingTime      << " s" << std::endl;
    std::cout << "  Peak time          : " << result.metrics.peakTime          << " s" << std::endl;
    std::cout << std::endl;

    // ── Convert Eigen vectors to std::vector for matplotlib-cpp ──────
    std::vector<double> timeVec     = toStdVector(result.simulation.time);
    std::vector<double> outputVec   = toStdVector(result.simulation.output);
    std::vector<double> controlVec  = toStdVector(result.simulation.control);
    std::vector<double> setpointVec(nSteps, setpoint);

    // ── Figure 1: Output vs Setpoint ─────────────────────────────────
    // Shows how the closed-loop output tracks the step reference.
    // Blue solid = actual output y(t); red dashed = setpoint.
    plt::figure();
    plt::plot(timeVec, outputVec,
        {{"label", "Output y(t)"}, {"color", "blue"}});
    plt::plot(timeVec, setpointVec,
        {{"label", "Setpoint"}, {"color", "red"}, {"linestyle", "--"}});
    plt::xlabel("Time (s)");
    plt::ylabel("Output");
    plt::title("Closed-Loop Step Response  |  G(s) = 2/(s+1)");
    plt::legend();

    // ── Figure 2: Control Signal ─────────────────────────────────────
    // Shows the control effort u(t) the PID sends to the plant.
    // A well-tuned controller produces a smooth curve that settles;
    // an aggressive one shows spikes or sustained oscillation.
    plt::figure();
    plt::plot(timeVec, controlVec,
        {{"label", "Control u(t)"}, {"color", "green"}});
    plt::xlabel("Time (s)");
    plt::ylabel("Control");
    plt::title("Control Signal  |  PID(Kp=2, Ki=1, Kd=0)");
    plt::legend();

    plt::show();

    return 0;
}
