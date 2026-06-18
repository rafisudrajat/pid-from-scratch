#include "ode.h"

double ODESolver::solveODEWithRalstonMethod(double (*f)(double,double),double x0, double y0, double stepSize){
    double k1 = f(x0,y0);
    double k2 = f(x0+3.0/4.0*stepSize, y0+3.0/4.0*k1*stepSize);
    return y0 + (k1/3.0 + 2.0/3.0*k2)*stepSize;
}

Eigen::VectorXd ODESolver::rk4Step(
    const std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>& f,
    double t,
    const Eigen::VectorXd& x,
    double stepSize)
{
    double h = stepSize;
    Eigen::VectorXd k1 = f(t, x);
    Eigen::VectorXd k2 = f(t + h / 2.0, x + (h / 2.0) * k1);
    Eigen::VectorXd k3 = f(t + h / 2.0, x + (h / 2.0) * k2);
    Eigen::VectorXd k4 = f(t + h, x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}