#include "ode.h"

double ODESolver::solveODEWithRalstonMethod(double (*f)(double,double),double x0, double y0, double stepSize){
    double k1 = f(x0,y0);
    double k2 = f(x0+3.0/4.0*stepSize, y0+3.0/4.0*k1*stepSize);
    return y0 + (k1/3.0 + 2.0/3.0*k2)*stepSize;
}