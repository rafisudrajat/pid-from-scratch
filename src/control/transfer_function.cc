#include "transfer_function.h"

TransferFunction::TransferFunction(const Eigen::VectorXd& numerator,
                                   const Eigen::VectorXd& denominator)
    : numerator_(numerator), denominator_(denominator)
{
    if (denominator_.size() == 0 || denominator_.isZero()) {
        throw std::invalid_argument("Denominator must be non-empty and non-zero");
    }
}

double TransferFunction::dcGain() const {
    return numerator_(numerator_.size() - 1) /
           denominator_(denominator_.size() - 1);
}

bool TransferFunction::isProper() const {
    return numerator_.size() <= denominator_.size();
}

bool TransferFunction::isStrictlyProper() const {
    return numerator_.size() < denominator_.size();
}
