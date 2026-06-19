#pragma once
#include <Eigen/Dense>
#include <complex>
#include <stdexcept>

class StateSpace;

/**
 * @brief SISO transfer function G(s) = num(s) / den(s).
 *
 * Stores numerator and denominator coefficients in descending power order
 * (highest degree first, constant term last). Provides queries that
 * control engineers ask of a plant before simulating it: DC gain and
 * properness.
 */
class TransferFunction {
public:
    /**
     * @brief Construct a transfer function from numerator and denominator
     *        coefficient vectors.
     *
     * @param numerator   Coefficients [b_m, ..., b_1, b_0] (descending powers).
     * @param denominator Coefficients [a_n, ..., a_1, a_0] (descending powers).
     * @throws std::invalid_argument If the denominator is empty or all-zero.
     */
    TransferFunction(const Eigen::VectorXd& numerator,
                     const Eigen::VectorXd& denominator);

    /**
     * @brief Steady-state (DC) gain G(0) = b_0 / a_0.
     *
     * Evaluates the transfer function at s = 0. All powers of s vanish,
     * leaving the ratio of the constant terms (last elements of each
     * coefficient vector). By the final-value theorem this equals the
     * unit-step steady-state value for a stable plant.
     *
     * @note Only valid for type 0 systems (no poles at the origin).
     *       For type 1+ systems (denominator has one or more s factors,
     *       i.e. a_0 = 0) the result is undefined (division by zero),
     *       because such plants have no finite steady-state value under
     *       a step input.
     *
     * @return The ratio of constant terms (last elements).
     */
    double dcGain() const;

    /**
     * @brief True if deg(num) <= deg(den) (physically realizable).
     */
    bool isProper() const;

    /**
     * @brief True if deg(num) < deg(den) (no direct feedthrough, D = 0).
     */
    bool isStrictlyProper() const;

    /**
     * @brief Roots of the denominator polynomial.
     *
     * The poles determine the natural modes of the system: a real pole
     * at −a produces a mode e^(−at); a complex pair ±jω produces
     * oscillation. Delegates to polynomialRoots (companion-matrix
     * eigensolver).
     *
     * @return Complex vector of poles (length = deg(den)).
     */
    Eigen::VectorXcd poles() const;

    /**
     * @brief Roots of the numerator polynomial.
     *
     * Zeros shape how inputs are transmitted through the system.
     * Delegates to polynomialRoots.
     *
     * @return Complex vector of zeros (length = deg(num)).
     */
    Eigen::VectorXcd zeros() const;

    /**
     * @brief True if all poles have strictly negative real part.
     *
     * A continuous-time LTI system is BIBO stable iff every pole
     * lies in the open left-half plane (Re(p) < 0).
     */
    bool isStable() const;

    /**
     * @brief Controllable canonical realization (A, B, C, D).
     *
     * Given a strictly proper transfer function
     *
     *           b_m s^m + ... + b_1 s + b_0
     *   G(s) = ——————————————————————————————   (m < n)
     *           a_n s^n + ... + a_1 s + a_0
     *
     * **Step 1 — Monic normalization.**  Divide every coefficient by
     * the leading denominator coefficient a_n so the denominator
     * becomes s^n + ā_{n-1} s^{n-1} + ... + ā_0  (where ā_i = a_i / a_n)
     * and similarly b̄_i = b_i / a_n.
     *
     * **Step 2 — Controllable canonical form.**  Build
     *
     *       ⎡ 0   1   0  ···  0  ⎤         ⎡ 0 ⎤
     *       ⎢ 0   0   1  ···  0  ⎥         ⎢ 0 ⎥
     *   A = ⎢ ⋮           ⋱   ⋮  ⎥,    B = ⎢ ⋮ ⎥,
     *       ⎢ 0   0   0  ···  1  ⎥         ⎢ 0 ⎥
     *       ⎣-ā_0 -ā_1 -ā_2 ··· -ā_{n-1}⎦  ⎣ 1 ⎦
     *
     *   C = [ b̄_0  b̄_1  ···  b̄_{m} 0 ··· 0 ],    D = [ 0 ].
     *
     * A is the companion matrix: ones on the super-diagonal, and the
     * negated monic denominator coefficients in ascending power order
     * along the bottom row.  B is the n-th standard basis vector.
     * C holds the normalized numerator coefficients in ascending power
     * order, zero-padded to length n.  D is zero because the transfer
     * function is strictly proper (deg(num) < deg(den)).
     *
     * **Why this works.**  The characteristic polynomial of the
     * companion matrix A is det(sI − A) = s^n + ā_{n-1} s^{n-1} + ... + ā_0,
     * which is exactly the monic denominator.  Therefore
     *   - eig(A) == poles(G),
     *   - −C·A⁻¹·B + D == G(0) == dcGain().
     *
     * @pre The transfer function must be strictly proper (m < n).
     */
    StateSpace toStateSpace() const;

    const Eigen::VectorXd& getNumerator() const { return numerator_; }
    const Eigen::VectorXd& getDenominator() const { return denominator_; }

private:
    Eigen::VectorXd numerator_;
    Eigen::VectorXd denominator_;
};
