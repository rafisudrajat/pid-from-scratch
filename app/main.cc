#include <iostream>
#include "factorial.h"
#include "matplotlibcpp.h"
#include "Eigen/Dense"
using Eigen::MatrixXd;

namespace plt = matplotlibcpp;

int main()
{
    plt::initialize_python();
    std::cout << factorial(5);

    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    plt::plot({1, 2, 3, 4});
    plt::show();
    return 0;
}
