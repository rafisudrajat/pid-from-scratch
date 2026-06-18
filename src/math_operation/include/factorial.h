#pragma once
#include <cstdint>

/**
 * @brief Compute the factorial of a non-negative integer.
 *
 * Uses recursive computation: n! = n * (n-1) * ... * 1, with 0! = 1.
 *
 * @param number Non-negative integer whose factorial is computed.
 * @return n! as a 32-bit unsigned integer.
 */
std::uint32_t factorial(std::uint32_t number);
