#pragma once
#include <Eigen/Dense>

/**
 * @brief Generate a vector of evenly spaced values over an interval.
 *
 * Returns @p count values uniformly distributed from @p start to @p end
 * (inclusive on both ends). If count == 1, returns a single-element
 * vector containing @p start.
 *
 * @param start First value of the sequence.
 * @param end   Last value of the sequence.
 * @param count Number of points to generate (must be >= 1).
 * @return Eigen::VectorXd of length @p count.
 */
Eigen::VectorXd linspace(double start, double end, int count);