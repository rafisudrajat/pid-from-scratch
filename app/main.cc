#include "factorial.h"
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    plt::initialize_python();
    std::cout << factorial(5);

    plt::plot({1, 2, 3, 4});
    plt::show();
    return 0;
}
