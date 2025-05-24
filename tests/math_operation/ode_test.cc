#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "ode.h"

std::vector<double> preComputedResults = 
    {1.00000, 3.277344, 3.101563, 2.347656, 2.140625, 2.855469, 4.117188, 4.800781, 3.031250};

double odeDummyFunction(double x, double y){
    double result = -2*std::pow(x,3) + 12*std::pow(x,2) - 20*x + 8.5;
    return result;
}

TEST(RalstonMethodTest, CompareWithPreComputedResults){
    // Define initial value of x0 and y0
    double x0 = 0.0;
    double odeSolverResult = 1.0;
    // Define step size (h)
    double stepSize = 0.5;
    // Create an instance of ODESolver and call the method
    ODESolver solver;
    for(int i=0; i<preComputedResults.size(); i++){
        double x = x0 + stepSize*i;
        ASSERT_NEAR(preComputedResults[i], odeSolverResult, 0.0001);        
        odeSolverResult = solver.solveODEWithRalstonMethod(&odeDummyFunction, x, odeSolverResult, stepSize);
    }
}