# ADR-001: Architecture Conventions

**Status:** Accepted  
**Date:** 2026-06-18

## Context

This project builds a PID control simulator from scratch in C++. Before implementing
the control modules (Phases 1–6), we fix the conventions every module depends on so
that design decisions are made once and not re-litigated mid-stream.

## Decisions

### 1. Polynomial coefficient convention: descending powers, constant term last

All polynomial coefficient vectors use **descending powers of s**. The highest-degree
coefficient is the first element; the constant term is the last.

```
numerator   = [b_m,   b_{m-1}, …, b_1, b_0]
denominator = [a_n,   a_{n-1}, …, a_1, a_0]
```

Example: `G(s) = (s + 3) / (s² + 3s + 2)` is stored as:

```
numerator   = [1, 3]
denominator = [1, 3, 2]
```

DC gain = last(num) / last(den) = 3/2 = 1.5.

### 2. Module split: `math_operation` (numerics) + `control` (control types)

| Library | Location | Contains |
|---|---|---|
| `MATH_OPERATION` | `src/math_operation/` | `linspace`, `polynomial` (eval + roots), `ODESolver` (Ralston RK2 + vector RK4) |
| `CONTROL` | `src/control/` (new) | `TransferFunction`, `StateSpace`, `PIDController`, `ClosedLoopSimulator`, `StepResponseMetrics` |

`CONTROL` links `MATH_OPERATION` publicly (it reuses `linspace`, `polynomial`, `rk4Step`).
Both are `STATIC` libraries with the same layout: headers in `include/`, sources as `.cc`.

### 3. State-space representation: `StateSpace` holding `(A, B, C, D)`

A SISO state-space system stores four Eigen matrices:

```
A : n×n     (state matrix)
B : n×1     (input matrix)
C : 1×n     (output matrix)
D : 1×1     (feedthrough matrix)
```

Two operations:

- `derivative(x, u) = A·x + B·u`  — the ODE right-hand side (what `rk4Step` calls)
- `output(x, u)     = C·x + D·u`  — the measured output (a scalar for SISO)

Dimensions are validated at construction; a mismatch throws.

### 4. TF → state-space realization: controllable canonical form

Given a strictly proper transfer function (after dividing both polynomials by the
leading denominator coefficient to make it monic):

```
G(s) = (b_{n-1} s^{n-1} + … + b_0) / (s^n + a_{n-1} s^{n-1} + … + a_0)
```

The controllable canonical form is:

```
      ┌ 0    1    0  … 0        ┐         ┌0┐
      │ 0    0    1  … 0        │         │0│
  A = │ ⋮              ⋱ ⋮      │ ,   B = │⋮│
      │ 0    0    0  … 1        │         │0│
      └ -a_0 -a_1 -a_2 … -a_{n-1} ┘      └1┘

  C = [b_0  b_1  …  b_{n-1}] ,   D = [0]
```

Note: the stored coefficients are in descending order, but the canonical form indexes
them ascending (`a_0, a_1, …`), so the implementation must **reverse** the coefficient
vector before filling the matrices.

**Falsifiable example (copy this into Step 3.2's test):**

For `G(s) = 1 / (s² + 3s + 2)`:

- Monic denominator: `s² + 3s + 2` → `a_0 = 2, a_1 = 3`
- Numerator (zero-padded to length n-1 = 1): `b_0 = 1, (no b_1, so effectively [1])`

```
A = [[ 0,  1],        B = [[0],       C = [[1, 0]],       D = [[0]]
     [-2, -3]]              [1]]
```

Self-checking invariants:
- `eigenvalues(A) == poles(G) == {-1, -2}`
- `G(0) = -C·A⁻¹·B + D = 1/2 == dcGain()`

### 5. PID controller: stateful, trapezoidal integral, backward-difference derivative

Constructor: `PIDController(kp, ki, kd, offset = 0)`.

Internal state: `integral` (running sum), `e_prev` (previous error), `time_prev`.

`compute(setpoint, measurement, time)`:

```
e     = setpoint - measurement
Ts    = time - time_prev          (guard: if Ts <= 0, use 1e-5)

P     = Kp * e
I[k]  = I[k-1] + Ki * (Ts/2) * (e + e_prev)     // trapezoidal rule
D     = Kd * (e - e_prev) / Ts                   // backward difference
u     = offset + P + I + D
```

First call: `e_prev = 0`.

`reset()`: clears `integral`, `e_prev`, `time_prev`.

Anti-windup (Step 4.2): when output limits `[u_min, u_max]` are set and `u` saturates,
clamp `u` and **hold** the integral (do not add the current step's contribution).

### 6. Scope

- **SISO** (single-input, single-output), continuous-time plants only.
- Strictly proper first (`D = 0`); proper (`D ≠ 0`) support may be added later.
- MIMO is out of scope.

## Naming conventions (match existing code)

- C++17
- `PascalCase` for types (`TransferFunction`, `StateSpace`, `PIDController`)
- `camelCase` for methods (`dcGain()`, `toStateSpace()`, `rk4Step()`)
- Headers in each module's `include/`, sources as `.cc`
