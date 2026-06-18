#pragma once
#include <functional>
#include <Eigen/Dense>

/**
 * @brief Numerical ODE solver collection.
 */
class ODESolver {
public:
    /**
     * @brief Perform one step of Ralston's method (2nd-order Runge-Kutta).
     *
     * Advances the solution of dy/dx = f(x, y) by one step using
     * Ralston's method with weights (1/3, 2/3) and sample point at 3/4.
     *
     * @param f  Right-hand side function f(x, y).
     * @param x0 Current x value.
     * @param y0 Current y value.
     * @param stepSize Step size h.
     * @return Approximate y value at x0 + h.
     */
    double solveODEWithRalstonMethod(double (*f)(double,double),double x0, double y0, double stepSize);

    /**
     * @brief Perform one step of the classic 4th-order Runge-Kutta method
     *        for a vector-valued ODE dx/dt = f(t, x).
     *
     * Computes four slope samples and combines them with weights
     * (1, 2, 2, 1)/6 to advance the state by one step. The method is
     * 4th-order accurate (global error O(h^4)) and integrates
     * polynomial-in-t slopes of degree <= 3 exactly (Simpson's rule).
     *
     * @param f        Right-hand side function f(t, x).
     * @param t        Current time.
     * @param x        Current state vector.
     * @param stepSize Step size h.
     * @return Approximate state vector at t + h.
     */
    Eigen::VectorXd rk4Step(
        const std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>& f,
        double t,
        const Eigen::VectorXd& x,
        double stepSize);
};
