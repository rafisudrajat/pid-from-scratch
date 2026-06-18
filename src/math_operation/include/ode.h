#pragma once

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
};
