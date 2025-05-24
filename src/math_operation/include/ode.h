#pragma once

class ODESolver {
public:
    // Define function to solve ODE with second order runge kutta method
    double solveODEWithRalstonMethod(double (*f)(double,double),double x0, double y0, double stepSize);
};
