#include "linspace.h"
#include <Eigen/Eigenvalues>

Eigen::VectorXd linspace(double start, double end, int count) {
    Eigen::VectorXd v(count);
    if (count == 1) {
        v(0) = start;
        return v;
    }
    double step = (end - start) / (count - 1);
    for (int i = 0; i < count; ++i) {
        v(i) = start + i * step;
    }
    return v;
}