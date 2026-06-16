# PID From Scratch in C++ — A Test-First ROADMAP

This roadmap builds a **PID control simulator from scratch in C++**, with **Eigen**
(linear algebra) and **matplotlib-cpp** (plotting) as the *only* dependencies. The end
goal is the workflow you sketched in [`python/pid.ipynb`](python/pid.ipynb), but with no
black box: **you type in a transfer function `G(s)`, the program closes a PID loop around
it, integrates the dynamics numerically, and plots the response** — and every numerical
mechanism underneath (ODE integration, transfer-function→state-space conversion, the
discretized PID, the feedback loop) is code *you* wrote and *you* tested.

The point is not just to get a plot. It is to **stop using MATLAB / `python-control` as an
oracle you don't understand** and instead re-derive the machinery yourself, proving each
piece against something you *can* check: closed-form control theory and the worked example
in the notebook.

It is written as a **test-driven (TDD) curriculum**. For every step you:

1. **Red** — write the test the step specifies, against a class/function that does not exist
   yet, and build/run it so you watch it **fail for the right reason** (won't compile / wrong
   number).
2. **Green** — write the smallest implementation that makes that test pass.
3. **(Refactor)** — clean it up while the test stays green.

You advance only when **both** are true: the test is *faithful* (it genuinely exercises the
behaviour — not a tautology you bent to make pass) **and** it is green.

---

## How we test: math oracles, anchored by control theory

Control systems are an unusually friendly domain for TDD, because so many answers are known
in closed form. Every test below uses one of these oracle types.

| Oracle | What it proves | Example |
|---|---|---|
| **Closed-form / hand-computed** | The formula is right on a case you worked out by hand | first-order step response `y(t)=K(1−e^(−t/τ))`; `roots([1,−3,2]) = {1,2}`; one PID step from hand-set gains |
| **Invariant / property** | A mathematical law the theory always implies | `eigenvalues(A) == poles(G)`; DC gain `G(0) = −CA⁻¹B + D`; integral action ⇒ **zero** steady-state error; final-value theorem `y_∞ = G(0)·r` |
| **Numerical convergence** | The discretization converges at its theoretical order | RK4 global error is `O(h⁴)`: halving `h` cuts the error ≈ **16×** |
| **Reference / golden** | Matches a trusted external result | the closed-loop trajectory reproduces [`python/pid.ipynb`](python/pid.ipynb)'s thermal-plant run |
| **Behavioural** | The whole loop actually controls | closed-loop output settles to the setpoint; overshoot shrinks as `Kd` rises |

**Why we still test, even though MATLAB exists.** You are *re-deriving* these results, not
calling a library. The closed-form column lets your tests assert the *true* answer (from the
math) instead of one you might mis-transcribe. The reference/golden column (the notebook) is
the end-to-end sanity check. Treat Phase 8 as the final verdict: the unit tests prove "each
formula is right," Phase 8 proves "the assembled simulator reproduces a known run."

---

## The faithfulness check (read this twice)

A test that asserts nothing, asserts something always-true, or that you edited to stop
failing, is **not** a pass — it is a silent hole. Before you trust a green bar, **sabotage
your own implementation** (flip a sign, drop a term, swap two coefficients) and confirm the
test goes red. Every step below names the specific sabotage to try.

---

## How to read each step

> **Goal** — the one-sentence outcome.
> **Control-theory specifics** — the exact formulas/conventions your test must assert, so you
> pin down the *true* answer rather than a guess.
> **Red — write the test** — the test file, the interface it pins down, the oracle type, the
> assertions and tolerance. End of Red = a test that **fails for the right reason** (usually
> the class/function doesn't exist, so it won't compile).
> **Green — make it pass** — the implementation to write.
> **Why it matters** — the transferable engineering/control skill.
> **Gate** — the command that must pass, plus the **faithfulness sabotage**.
> **Commit** — a [Conventional Commits](https://www.conventionalcommits.org/) message. One
> green gate = one commit.

**Conventions for this codebase (match the existing code):**
- **C++17**, `PascalCase` types, `camelCase` methods (e.g. `solveODEWithRalstonMethod`),
  headers in each module's `include/`, sources as `.cc`.
- Each module under `src/<name>/` builds a **STATIC library** via its own `CMakeLists.txt`;
  tests mirror it under `tests/<name>/` and link **GoogleTest** + that library. The exact CMake
  recipe for adding each file/module is in **§Wiring** below.
- Eigen is included as `#include "Eigen/Dense"` from the bundled
  `src/external_lib/eigen-lib-3.4.0`. Use `Eigen::VectorXd` / `Eigen::MatrixXd` for real data
  and `Eigen::VectorXcd` for poles/zeros.
- **Polynomial coefficient convention (fix it once, use everywhere): descending powers.**
  `numerator = [b_m, …, b_1, b_0]`, `denominator = [a_n, …, a_1, a_0]`. The constant term is
  the **last** element. Document this in the ADR (Step 0.3) and never deviate.
- **Tolerances:** exact algebra on `double` → `ASSERT_NEAR(…, 1e-9)`; analytic comparisons →
  `1e-6`; RK4-integrated trajectories → `1e-4`; convergence-ratio checks → within ~10% of 16.
- Scope is **SISO** (single-input single-output), continuous-time plants. MIMO is out of scope.
- **Build wiring is manual** (no globbing). Every step that adds a file also implies a CMake
  edit — see the next section. Forgetting it is the `No tests were found` / `Eigen/Dense: No
  such file or directory` class of error.

---

## Wiring new code into the build (you will do this at almost every step)

CMake here does **not** auto-discover files: test executables are listed one by one and the
library source list is explicit. The two failure modes this prevents are *"my new test never
runs"* (forgot to register it) and *"Eigen/Dense: No such file or directory"* (the include
path doesn't reach the target). Each lettered recipe below is referenced from the steps as
**§Wiring A–E**.

**Top-level testing must stay on.** `enable_testing()` lives in the root
[`CMakeLists.txt`](CMakeLists.txt); without it `ctest` prints *"No tests were found"* even
though the test binaries built fine. That is the canonical home — don't rely on the copy in a
subdirectory.

**A — Register a new test in an existing test module** (e.g. `linspace_test.cc` under
[`tests/math_operation/`](tests/math_operation/CMakeLists.txt)): add three lines to that
module's `CMakeLists.txt`:
```cmake
add_executable(linspace_test linspace_test.cc)
target_link_libraries(linspace_test GTest::gtest_main MATH_OPERATION)
gtest_discover_tests(linspace_test)
```

**B — Add a source file to the `math_operation` library** (e.g. `linspace.cc`): append the
`.cc` to `LIBRARY_SOURCES` and the `.h` to `LIBRARY_HEADERS` in
[`src/math_operation/CMakeLists.txt`](src/math_operation/CMakeLists.txt). That list is typed
out by hand — a new file not added there simply isn't compiled into the library.

**C — Make Eigen reachable from a library's consumers (the subtle one).**
[`src/CMakeLists.txt`](src/CMakeLists.txt)'s `include_directories(... eigen-lib-3.4.0)` is
**directory-scoped to `src/` and does NOT propagate to `tests/`** (that's why
[`app/CMakeLists.txt`](app/CMakeLists.txt) adds the Eigen path itself). The first moment a
library's **public header** includes `Eigen/Dense` is **Step 1.1** (`linspace.h` returns
`Eigen::VectorXd`); from then on every test that includes that header needs Eigen on its path.
Fix it once, on the library target, so it propagates transitively to anything that links it:
```cmake
target_include_directories(MATH_OPERATION PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0)
```
Do the same on the new `control` library. After this, any target that links the library gets
Eigen for free — no per-test include paths needed.

**D — A test that includes Eigen directly but links no Eigen-exposing library** (e.g.
`eigen_matchers_test.cc` in Step 0.2, which links only GoogleTest): give *that* target the path
explicitly:
```cmake
target_include_directories(eigen_matchers_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0)
```

**E — Stand up a whole new module (the `control` library, first needed in Phase 2):**
1. Create `src/control/CMakeLists.txt`, modeled on `math_operation`'s: its own `STATIC` lib
   with `target_include_directories(... PUBLIC include + Eigen path)` per **C**, and
   `target_link_libraries(CONTROL PUBLIC MATH_OPERATION)` (it reuses `linspace`/`polynomial`/
   `rk4Step`).
2. Add `add_subdirectory(control)` to [`src/CMakeLists.txt`](src/CMakeLists.txt).
3. Create `tests/control/CMakeLists.txt` (one `add_executable`/`gtest_discover_tests` per test
   file, per **A**, linking the `control` lib).
4. Add `add_subdirectory(control)` to [`tests/CMakeLists.txt`](tests/CMakeLists.txt).
5. The shared `tests/support/` directory (created back in Step 0.2) is wired the same way:
   create `tests/support/CMakeLists.txt` registering `eigen_matchers_test` (per **A**, plus
   **D** for its Eigen path), add `add_subdirectory(support)` to
   [`tests/CMakeLists.txt`](tests/CMakeLists.txt), and expose `eigen_matchers.h` to other test
   targets — either by adding `${CMAKE_SOURCE_DIR}/tests/support` to their include dirs or
   (cleaner) as a header-only `INTERFACE` library they link.

**After any wiring change**, re-run the full pipeline from the project root and confirm your new
test name actually appears in the ctest listing — *don't* trust a `100% passed` that only ran
the pre-existing tests:
```
cmake -S . -B build && cmake --build build && ctest --test-dir build
```

---

## Phase 0 — Foundations: the harness that makes Red/Green possible

This phase writes *no control code*. It makes the build/test loop trustworthy and records the
design decisions everything else assumes. (GoogleTest is already wired — `factorial_test`,
`ode_test` — and the bundled Eigen now builds, so we start from a green base.)

### Step 0.1 — Green pipeline + one-command test run
**Goal:** `cmake --build` succeeds and `ctest` runs the existing suite green, so every later
Red/Green is measured against a known-good baseline.
**Red — write the test:** none new — *run* the existing suite. From the **project root** (the
directory containing `CMakeLists.txt`), targeting a fresh `build/` subdirectory:
`cmake -S . -B build && cmake --build build && ctest --test-dir build`. If the build succeeds
but `ctest` prints **`No tests were found!!!`**, that's your red — the cause is a missing
`enable_testing()` in the **root** [`CMakeLists.txt`](CMakeLists.txt). (It was present only in
[`tests/math_operation/CMakeLists.txt`](tests/math_operation/CMakeLists.txt), whose scope
`ctest` does not descend into unless testing is enabled at the top level first.)
**Green — make it pass:** ensure `enable_testing()` is active at the top level and `ctest`
discovers `factorial_test` and `ode_test`. Confirm the bundled Eigen path compiles
(`app/main.cc` includes `Eigen/Dense`).
**Why it matters:** TDD is impossible without a trustworthy "all green" you can return to.
**Gate:** `ctest --test-dir build` reports all tests passed. Faithfulness: temporarily break
one `EXPECT_EQ` in `factorial_test.cc` → `ctest` goes red → revert.
**Commit:** `chore(build): verify ctest pipeline green on bundled Eigen`

### Step 0.2 — Eigen test matchers (built test-first)
**Goal:** Build the comparison helpers *every later test* will call, so you assert
"these two vectors/matrices/pole-sets are close" in one line instead of looping by hand.
**What we are building.** GoogleTest's `ASSERT_NEAR` is scalar-only. You need three helpers,
returning `::testing::AssertionResult` so failures print *where* and *how far*:
- `expectVectorNear(a, b, tol)` — element-wise `|aᵢ−bᵢ| ≤ tol` for `Eigen::VectorXd`; on
  failure, report the worst index and its error.
- `expectMatrixNear(A, B, tol)` — same for `Eigen::MatrixXd`.
- `expectComplexSetNear(a, b, tol)` — for `Eigen::VectorXcd` **as an unordered set** (poles
  come back in solver-dependent order): sort both by `(real, imag)` then compare. Needed so a
  poles test isn't sensitive to ordering.
**Red — write the test:** `tests/support/eigen_matchers_test.cc` — *meta-tests* of the tools:
- `test_vector_near_passes_and_fails`: `expectVectorNear(v, v, 1e-9)` passes; `v` vs `v +
  unit·1.0` fails, and the message contains the worst index and the error `1.0`.
- `test_complex_set_is_order_insensitive`: `{1+0i, 2+0i}` vs `{2+0i, 1+0i}` passes;
  `{1,2}` vs `{1,3}` fails.
- Build → **red** (`eigen_matchers.h` doesn't exist).
**Green — make it pass:** `tests/support/eigen_matchers.h` (header-only) with the three
helpers; add a `tests/support` include path so test files do `#include "eigen_matchers.h"`.
**Wire it** per **§Wiring D** (the meta-test includes `Eigen/Dense` directly, so it needs the
Eigen path on its own target) and **§Wiring E.5** (put `tests/support` on the include path and
register the new test directory in [`tests/CMakeLists.txt`](tests/CMakeLists.txt)).
**Why it matters:** Your oracles are only as trustworthy as the tools that apply them; meta-
testing them first means every later "green" means something.
**Gate:** the matcher meta-tests pass. Faithfulness: make `expectVectorNear` `return
::testing::AssertionSuccess()` unconditionally → the failure half of its meta-test goes red.
**Commit:** `test(support): add Eigen vector/matrix/complex-set matchers`

### Step 0.3 — Architecture decision record (design checkpoint)
**Goal:** Decide, once, how the pieces map to C++ types so Phases 1–6 are mechanical.
**Decisions to record (in `doc/adr-001-architecture.md`):**
- **Coefficient convention:** descending powers, constant term last (as above).
- **Module split:** numerical primitives (`linspace`, polynomial, RK4) live in the existing
  **`math_operation`** library; control types (`TransferFunction`, `StateSpace`,
  `PIDController`, `ClosedLoopSimulator`, `StepResponseMetrics`) live in a **new `control`**
  library under `src/control/` mirroring the `math_operation` layout.
- **State-space representation:** a `StateSpace` holding `Eigen::MatrixXd A, B, C, D`, with
  `derivative(x,u) = A·x + B·u` and `output(x,u) = C·x + D·u`.
- **TF→SS realization:** **controllable canonical form** (companion `A`). Write the exact
  matrix layout into the ADR as a falsifiable example (the 2×2 case below).
- **PID state:** the controller is *stateful* (holds `integral`, `e_prev`, `time_prev`) and
  exposes `reset()`; `compute(setpoint, measurement, time)` mirrors the notebook's signature.
- **Scope:** SISO, continuous-time, strictly-proper-first (support proper `D≠0` later).
**Gate:** ADR written; the canonical-form example is concrete enough to copy into Step 3.2's
test. **Commit:** `docs(adr): record coefficient, module, and state-space conventions`

---

## Phase 1 — Numerical primitives

Pure functions in the `math_operation` library — the easiest Red/Green wins, and the
foundation the plant simulation stands on.

### Step 1.1 — `linspace`
**Goal:** The time-grid generator (the C++ analog of the notebook's `np.linspace`).
**Control-theory specifics:** `linspace(start, stop, n)` returns `n` points **including both
endpoints**, equally spaced by `h = (stop − start)/(n − 1)`. `n == 1` returns `{start}`.
**Red — write the test:** `tests/math_operation/linspace_test.cc`:
- `test_endpoints_and_count` *(closed-form)*: `linspace(0, 10, 50)` has size 50, `[0]==0`,
  `[49]==10`.
- `test_uniform_spacing` *(closed-form)*: consecutive differences all equal `10/49`,
  `1e-12`.
- `test_single_point` *(edge)*: `linspace(3, 7, 1)` is `{3}`.
- Build → **red**.
**Green — make it pass:** `math_operation/linspace.{h,cc}` returning `Eigen::VectorXd`. **Wire
it** per **§Wiring A + B**, and — because `linspace.h` is the **first** `math_operation` header
to include `Eigen/Dense` — apply **§Wiring C** now (make Eigen a `PUBLIC` include of
`MATH_OPERATION`), or `linspace_test` will fail with `Eigen/Dense: No such file or directory`.
**Why it matters:** Every simulation runs on a time grid; off-by-one on the endpoint silently
shifts every later sample.
**Gate:** the linspace tests pass. Faithfulness: divide by `n` instead of `n−1` → the spacing
test (and `[49]==10`) go red.
**Commit:** `feat(math): add linspace time-grid generator`

### Step 1.2 — Polynomial evaluation and roots
**Goal:** Evaluate a polynomial and find its roots — the kernel behind DC gain, poles, and
zeros.
**Control-theory specifics:**
- **Eval (Horner, descending coeffs):** `p(x) = ((c₀·x + c₁)·x + c₂)…`. Provide a real
  overload and a `std::complex<double>` overload (poles are complex).
- **Roots via the companion matrix.** Normalize to monic by dividing by the leading
  coefficient; build the `n×n` companion matrix of `sⁿ + d_{n−1}sⁿ⁻¹ + … + d₀`, whose
  **eigenvalues are exactly the roots**. Use `Eigen::EigenSolver<Eigen::MatrixXd>` and return
  `.eigenvalues()` as `Eigen::VectorXcd`. (This is the same companion structure as the
  controllable-form `A` in Step 3.2 — building it here pays off twice.)
**Red — write the test:** `tests/math_operation/polynomial_test.cc`:
- `test_eval_hand_values` *(closed-form)*: `[1,−3,2]` (i.e. `s²−3s+2`) at `s=0` is `2`, at
  `s=1` is `0`, at `s=2` is `0`. `1e-9`.
- `test_real_roots` *(closed-form)*: `roots([1,−3,2]) == {1, 2}` (use
  `expectComplexSetNear`, `1e-9`).
- `test_complex_roots` *(closed-form)*: `roots([1,0,1])` (i.e. `s²+1`) `== {+i, −i}`.
- `test_roots_match_eval` *(invariant)*: each returned root `r` satisfies `|p(r)| < 1e-8`.
- Build → **red**.
**Green — make it pass:** `math_operation/polynomial.{h,cc}` (`polynomialEval`,
`polynomialRoots`). Wire it per **§Wiring A + B**.
**Why it matters:** Poles/zeros and the DC gain all reduce to these two operations; the
companion-matrix trick is the bridge from algebra to Eigen's eigensolver.
**Gate:** the polynomial tests pass. Faithfulness: drop the monic normalization (skip dividing
by the leading coeff) → `test_real_roots` goes red on a non-monic input like `[2,−6,4]`.
**Commit:** `feat(math): add polynomial eval + roots (companion matrix)`

### Step 1.3 — Vector-valued RK4 integrator
**Goal:** One classic 4-stage Runge–Kutta step for a **state-vector** ODE `ẋ = f(t, x)`,
generalizing the existing scalar Ralston RK2 so it can integrate `ẋ = A·x + B·u`.
**Control-theory specifics (classic RK4 — assert these exact weights):**
`k₁ = f(t, x)`, `k₂ = f(t + h/2, x + (h/2)k₁)`, `k₃ = f(t + h/2, x + (h/2)k₂)`,
`k₄ = f(t + h, x + h·k₃)`, `x_next = x + (h/6)(k₁ + 2k₂ + 2k₃ + k₄)`. RK4 integrates
`ẋ = f(t)` **exactly for polynomials of degree ≤ 3** (it reduces to Simpson's rule), and its
**global** error is `O(h⁴)`.
**Red — write the test:** add to `tests/math_operation/ode_test.cc` (keep the existing Ralston
test):
- `test_rk4_exact_on_cubic` *(closed-form)*: for `ẋ = 3t²` with `x(0)=0`, one step of size
  `h=0.5` gives **exactly** `x=t³=0.125` (Simpson-exact for cubics), `1e-12`.
- `test_rk4_matches_exponential` *(closed-form)*: `ẋ = x`, `x(0)=1`, integrate to `t=1` with
  small `h`; result ≈ `e`, `1e-4`.
- `test_rk4_fourth_order_convergence` *(convergence — the point)*: integrate `ẋ = x` to a
  fixed `t` at step `h` and at `h/2`; the error ratio is `≈ 16` (within 10%).
- `test_rk4_vector_state` *(contract)*: a 2-D linear system `ẋ = A x` integrates each
  component correctly (e.g. a rotation/decay matrix vs its matrix-exponential value, `1e-4`).
- Build → **red**.
**Green — make it pass:** extend `ODESolver` in `math_operation/ode.{h,cc}` with
`Eigen::VectorXd rk4Step(const std::function<Eigen::VectorXd(double,const Eigen::VectorXd&)>& f,
double t, const Eigen::VectorXd& x, double stepSize)`. `ode.h` now needs `#include <functional>`
and `#include "Eigen/Dense"`; this relies on the `MATH_OPERATION` public-Eigen wiring from
**§Wiring C** (done in Step 1.1) — without it, the existing `ode_test` will stop compiling.
**Why it matters:** This is the engine that advances the plant. The convergence test is what
*proves* it's genuinely 4th-order rather than "looks about right."
**Gate:** the RK4 tests pass. Faithfulness: change the weights `(1,2,2,1)/6` to `(1,1,1,1)/4`
→ `test_rk4_exact_on_cubic` and the convergence test go red.
**Commit:** `feat(math): add vector RK4 integrator (ODESolver::rk4Step)`

---

## Phase 2 — Transfer functions

The user-facing input. A `TransferFunction` stores numerator/denominator coefficients and
answers the questions control engineers ask of `G(s)`.

### Step 2.1 — `TransferFunction` construction, DC gain, properness
**Goal:** Build `G(s) = num(s)/den(s)` and read off its DC (steady-state) gain.
**Control-theory specifics:**
- Store `Eigen::VectorXd numerator, denominator` (descending powers). Reject an empty or
  all-zero denominator.
- **DC gain** `G(0) = b₀/a₀` = (last numerator coeff)/(last denominator coeff). By the
  final-value theorem this is the unit-step **final value**.
- **Proper** iff `deg(num) ≤ deg(den)` (i.e. `numerator.size() ≤ denominator.size()`);
  **strictly proper** iff `<`.
**Red — write the test:** `tests/control/transfer_function_test.cc`:
- `test_dc_gain_first_order` *(closed-form)*: `G = 5/(2s+1)` → `num=[5]`, `den=[2,1]` →
  `dcGain()==5`.
- `test_dc_gain_general` *(closed-form)*: `G=(s+3)/(s²+2s+6)` → `dcGain()==3/6==0.5`.
- `test_properness` *(contract)*: strictly-proper, proper-equal-degree, and improper cases
  classify correctly.
- `test_rejects_zero_denominator` *(edge)*: constructing with an all-zero denominator throws.
- Build → **red**.
**Green — make it pass:** this is the **first `control` code**, so before it can compile you must
**stand up the new `control` library and `tests/control/` directory per §Wiring E** (new
`src/control/CMakeLists.txt` + `add_subdirectory(control)` in both
[`src/CMakeLists.txt`](src/CMakeLists.txt) and [`tests/CMakeLists.txt`](tests/CMakeLists.txt)).
Then write `control/transfer_function.{h,cc}` (`dcGain`, `isProper`, `isStrictlyProper`).
**Why it matters:** DC gain is the first sanity check on any loop ("where does it settle?");
properness decides whether a state-space realization even exists.
**Gate:** the TF tests pass. Faithfulness: compute DC gain from the **leading** coeffs
(`num[0]/den[0]`) instead of the constant terms → `test_dc_gain_first_order` goes red.
**Commit:** `feat(control): add TransferFunction (dc gain, properness)`

### Step 2.2 — Poles and zeros
**Goal:** Expose `poles()` and `zeros()` of `G(s)`.
**Control-theory specifics:** `poles() = polynomialRoots(denominator)`, `zeros() =
polynomialRoots(numerator)` (Step 1.2). Stability is "all poles have negative real part" — a
convenience predicate you can add.
**Red — write the test:** add to `transfer_function_test.cc`:
- `test_poles_of_second_order` *(closed-form)*: `G=1/(s²+3s+2)` → poles `{−1, −2}`
  (`expectComplexSetNear`, `1e-9`).
- `test_complex_poles` *(closed-form)*: `G=1/(s²+2s+5)` → poles `{−1±2i}`.
- `test_zeros` *(closed-form)*: `G=(s²−1)/(s²+2s+2)` → zeros `{−1, +1}`.
- `test_stability_predicate` *(invariant)*: the second-order stable plant is reported stable;
  a plant with a right-half-plane pole is not.
- Build → **red**.
**Green — make it pass:** `TransferFunction::poles()`, `zeros()`, `isStable()`.
**Why it matters:** Poles *are* the dynamics; verifying them against the denominator roots is
the invariant that ties the whole transfer-function representation together.
**Gate:** the poles/zeros tests pass. Faithfulness: have `poles()` factor the **numerator** by
mistake → `test_poles_of_second_order` goes red.
**Commit:** `feat(control): add poles, zeros, and stability check`

---

## Phase 3 — State space (the simulatable form)

A transfer function isn't directly integrable; its **state-space realization** is. This phase
converts `G(s)` to `(A,B,C,D)` and simulates its open-loop response.

### Step 3.1 — `StateSpace` type
**Goal:** A container for `(A,B,C,D)` with the two operations a simulator needs.
**Control-theory specifics:** `derivative(x,u) = A·x + B·u` (an `n`-vector), `output(x,u) =
C·x + D·u` (a scalar for SISO). Validate dimensions at construction (`A` square `n×n`, `B`
`n×1`, `C` `1×n`, `D` `1×1`).
**Red — write the test:** `tests/control/state_space_test.cc`:
- `test_derivative_and_output` *(closed-form)*: with hand-set tiny `(A,B,C,D)` and a chosen
  `(x,u)`, `derivative` and `output` equal the hand-computed vectors/scalars, `1e-12`.
- `test_dimension_validation` *(edge)*: mismatched shapes throw.
- Build → **red**.
**Green — make it pass:** `control/state_space.{h,cc}`.
**Why it matters:** This is the clean boundary the integrator talks to; getting `A·x + B·u`
right (and dimension-checked) prevents a class of silent shape bugs.
**Gate:** the state-space tests pass. Faithfulness: drop the `+ D·u` term in `output` →
`test_derivative_and_output` goes red on a case with `D≠0`.
**Commit:** `feat(control): add StateSpace (derivative + output)`

### Step 3.2 — Transfer function → controllable canonical form
**Goal:** `TransferFunction::toStateSpace()` producing the controllable canonical realization.
**Control-theory specifics (assert this exact layout).** For a strictly proper, monic-
normalized `G(s) = (b_{n−1}sⁿ⁻¹+…+b₀)/(sⁿ + a_{n−1}sⁿ⁻¹+…+a₀)` (divide both by the leading
denominator coeff first):
```
      ┌ 0    1    0  … 0   ┐         ┌0┐
      │ 0    0    1  … 0   │         │0│
  A = │ ⋮              ⋱ ⋮ │ ,   B = │⋮│ ,  C = [b₀  b₁ … b_{n−1}] ,  D = 0
      │ 0    0    0  … 1   │         │0│
      └ −a₀ −a₁ −a₂ … −a_{n−1} ┘     └1┘
```
Two invariants make this self-checking: **`eigenvalues(A) == poles(G)`** and **`G(0) =
−C·A⁻¹·B + D`** (steady-state gain from the realization equals the DC gain from Step 2.1).
**Red — write the test:** add to `state_space_test.cc`:
- `test_eigenvalues_equal_poles` *(invariant — the crux)*: for `G=1/(s²+3s+2)`,
  `eigenvalues(ss.A) == G.poles() == {−1,−2}` (`expectComplexSetNear`, `1e-9`).
- `test_realization_dc_gain` *(invariant)*: `−C·A⁻¹·B + D == G.dcGain()`, `1e-9`, for both a
  first-order and a second-order `G`.
- `test_first_order_layout` *(closed-form)*: `G=K/(τs+1)` realizes to `A=[−1/τ]`, `B=[1]`,
  `C=[K/τ]`, `D=[0]`.
- Build → **red**.
**Green — make it pass:** `TransferFunction::toStateSpace()` (reverse coeffs to ascending,
normalize to monic, fill the companion `A`, set `B`, `C`, `D`).
**Why it matters:** This is *the* conversion that makes a transfer function simulatable; the
eigenvalues-equal-poles invariant is an exceptionally strong, sign-sensitive check.
**Gate:** the realization tests pass. Faithfulness: put `+aᵢ` instead of `−aᵢ` in the bottom
row of `A` → `test_eigenvalues_equal_poles` goes red (poles flip sign).
**Commit:** `feat(control): add TF→state-space (controllable canonical form)`

### Step 3.3 — Open-loop step response via RK4
**Goal:** Simulate `ẋ = A·x + B·u`, `y = C·x + D·u` for a constant input over a time grid.
**Control-theory specifics:** zero-order hold — `u` is held constant across each RK4 step
(Step 1.3). For a unit step on `G=K/(τs+1)`, the exact response is
**`y(t) = K(1 − e^(−t/τ))`** (passes through `0.632K` at `t=τ`, settles at `K`).
**Red — write the test:** add to `state_space_test.cc`:
- `test_first_order_step_matches_analytic` *(closed-form)*: simulate `G=2/(s+1)` (`K=2,τ=1`)
  to a unit step on a fine grid; every sample matches `2(1−e^(−t))`, `1e-4`. (Sabotage anchor.)
- `test_final_value_is_dc_gain` *(invariant)*: the last sample ≈ `G.dcGain()·u`, `1e-3`.
- `test_zero_input_zero_state_stays_zero` *(edge)*: `x₀=0, u=0` ⇒ `y≡0`.
- Build → **red**.
**Green — make it pass:** a `simulateStep(StateSpace, u, timeVector, x0)` helper (free function
in `control`, or a method on `StateSpace`) that threads `rk4Step` across the grid and records
`y` at each point.
**Why it matters:** Proving the open-loop simulator against the textbook first-order response
means any later closed-loop discrepancy is the *controller's* doing, not the integrator's.
**Gate:** the step-response tests pass. Faithfulness: integrate `A·x` only (drop `B·u`) →
`test_first_order_step_matches_analytic` goes red (output stays at 0).
**Commit:** `feat(control): add open-loop state-space step simulation`

---

## Phase 4 — The PID controller

The discretized PID from [`python/pid.ipynb`](python/pid.ipynb) §"Discretized PID Equation",
ported to a stateful C++ class.

### Step 4.1 — Discretized P, I, D
**Goal:** `PIDController::compute(setpoint, measurement, time)` implementing the notebook's
equation (5).
**Control-theory specifics (the exact discretization — match the notebook):** with error
`e[k] = setpoint − measurement` and step `Tₛ = time − time_prev`:
- **Proportional:** `P = Kp·e[k]`.
- **Integral (trapezoidal accumulator):** `I[k] = I[k−1] + Ki·(Tₛ/2)·(e[k] + e[k−1])`.
- **Derivative (backward difference):** `D = Kd·(e[k] − e[k−1])/Tₛ`.
- **Output:** `u[k] = offset + P + I[k] + D` (the notebook's `offset` is the operating-point
  bias; default it to `0`). First call: `e_prev=0`, guard `Tₛ ≤ 0` by substituting a tiny
  `Tₛ` (the notebook uses `1e-5`) so the D term doesn't divide by zero. `reset()` clears
  `I`, `e_prev`, `time_prev`.
**Red — write the test:** `tests/control/pid_controller_test.cc`:
- `test_pure_proportional` *(closed-form)*: `Kp=2, Ki=Kd=0`; one `compute(setpoint=1,
  measurement=0, …)` returns `2·1 = 2`.
- `test_first_step_hand_value` *(closed-form)*: `Kp,Ki,Kd` set, first call with known
  `e, Tₛ`; `u` equals the hand-computed `P + I + D` (with `e_prev=0`), `1e-9`.
- `test_integral_accumulates_trapezoidally` *(closed-form)*: two calls with **constant** `e`
  and fixed `Tₛ` ⇒ `I` after step 2 equals `Ki·Tₛ·e + Ki·(Tₛ/2)·e`… i.e. the hand-rolled
  trapezoid sum; assert the exact accumulator value, `1e-9`.
- `test_derivative_on_error_change` *(closed-form)*: with `Kp=Ki=0`, a jump in `e` from `0`
  to `Δ` over `Tₛ` gives `D = Kd·Δ/Tₛ`.
- `test_reset_clears_state` *(invariant)*: after `reset()`, the integral and `e_prev` are
  zero (a repeated input sequence reproduces the first run).
- Build → **red**.
**Green — make it pass:** `control/pid_controller.{h,cc}` (constructor `(kp, ki, kd, offset=0)`,
`compute`, `reset`).
**Why it matters:** This *is* the controller. The trapezoidal/backward-difference choices are
exactly where a "looks plausible" implementation quietly diverges from the math — the hand
values pin them down.
**Gate:** the PID tests pass. Faithfulness: replace the trapezoidal integral with a rectangular
one (`Ki·Tₛ·e[k]`) → `test_integral_accumulates_trapezoidally` goes red.
**Commit:** `feat(control): add discretized PID controller (P, I, D)`

### Step 4.2 — Integrator anti-windup (clamping)
**Goal:** Optional but realistic: stop the integral from accumulating while the output is
saturated.
**Control-theory specifics:** given output limits `[u_min, u_max]`, after computing `u`, if it
saturates, **clamp `u`** and **hold the integral accumulator** (don't add this step's
contribution) — conditional integration. Without limits the controller behaves exactly as in
4.1.
**Red — write the test:** add to `pid_controller_test.cc`:
- `test_output_clamped` *(closed-form)*: with tight limits, a large error yields exactly
  `u_max`.
- `test_no_windup` *(behavioural)*: drive the controller into saturation for many steps, then
  reduce the error; the response recovers without the long overshoot an unclamped integral
  would produce (compare integral magnitude vs the unclamped controller).
- `test_disabled_limits_match_step_4_1` *(invariant)*: with infinite limits, outputs equal the
  Step 4.1 controller bit-for-bit.
- Build → **red**.
**Green — make it pass:** add `setOutputLimits(min, max)` and conditional integration to
`compute`.
**Why it matters:** Windup is the single most common reason a hand-rolled PID "works in theory,
oscillates in practice"; conditional integration is the standard fix.
**Gate:** the anti-windup tests pass. Faithfulness: clamp the output but **still** accumulate
the integral → `test_no_windup` goes red.
**Commit:** `feat(control): add PID output clamping + anti-windup`

---

## Phase 5 — Closed-loop simulation

Wire setpoint → error → PID → plant → measurement → feedback. This is the deliverable the user
asked for: a transfer function in, a controlled response out.

### Step 5.1 — `ClosedLoopSimulator`
**Goal:** Run the unity-feedback loop over a time grid and return `(time, output, control)`.
**Control-theory specifics (the loop, matching the notebook's structure).** For each step `k`
on the grid:
1. measure `y[k] = C·x[k] + D·u[k−1]`,
2. `u[k] = controller.compute(setpoint, y[k], t[k])`,
3. integrate the plant over `[t[k], t[k+1]]` with `rk4Step`, holding `u[k]` constant (ZOH),
4. record `y`, `u`.
**Red — write the test:** `tests/control/closed_loop_simulator_test.cc`:
- `test_result_shapes` *(contract)*: outputs have the same length as the time grid; the first
  recorded output is `C·x₀`.
- `test_settles_near_setpoint_with_pi` *(behavioural)*: a PI controller on a stable
  first-order plant drives `y` to within 1% of the setpoint by the end of a long-enough run.
- `test_zero_gains_is_open_loop` *(invariant)*: with `Kp=Ki=Kd=0`, `u≡offset` and the output
  matches the open-loop simulation from Step 3.3.
- Build → **red**.
**Green — make it pass:** `control/closed_loop_simulator.{h,cc}` with a `SimulationResult`
(`Eigen::VectorXd time, output, control`) returned by `simulate(plant, controller, setpoint,
timeVector, x0)`.
**Why it matters:** First full subsystem; the open-loop-equivalence invariant proves the loop
wiring adds nothing spurious when the controller is silent.
**Gate:** the closed-loop tests pass. Faithfulness: compute the error as `y − setpoint`
(positive feedback) → `test_settles_near_setpoint_with_pi` goes red (the loop diverges or
settles at the wrong value).
**Commit:** `feat(control): add closed-loop PID simulator`

### Step 5.2 — Steady-state correctness (the controller's signature behaviours)
**Goal:** Prove the two textbook steady-state results that distinguish P from PI control.
**Control-theory specifics (closed-form steady states for a first-order plant
`G=K/(τs+1)` under unity feedback and a unit-step setpoint):**
- **P-only** (`Kp`, `Ki=Kd=0`): closed loop `T=KpK/(τs+1+KpK)`, so `y_∞ = KpK/(1+KpK)` and
  the **steady-state error is `e_∞ = 1/(1+KpK) ≠ 0`** — a permanent offset. With `K=1,
  Kp=9`: `y_∞=0.9`, `e_∞=0.1`.
- **PI** (`Ki>0`): the integrator forces `e_∞ → 0`; `y_∞ → setpoint`.
**Red — write the test:** add to `closed_loop_simulator_test.cc`:
- `test_p_only_steady_state_offset` *(closed-form)*: `K=1, Kp=9`, simulate long enough; final
  output ≈ `0.9` and steady-state error ≈ `0.1`, `1e-2`. (Sabotage anchor.)
- `test_integral_eliminates_offset` *(invariant)*: adding `Ki>0` drives the final error below
  `1e-3` on the same plant.
- Build → **red**.
**Green — make it pass:** no new code if Phase 5.1 is correct — these are *validation* tests of
the existing simulator (write them, watch them pass; if they don't, the loop math is wrong).
**Why it matters:** "Integral action removes steady-state error" is the defining reason PID
beats pure P; encoding it as a test turns control intuition into a regression guard.
**Gate:** both tests pass. Faithfulness: zero out the integral contribution inside `compute`
→ `test_integral_eliminates_offset` goes red.
**Commit:** `test(control): pin P-offset and PI zero-offset steady states`

---

## Phase 6 — Step-response metrics

Quantify a response so "is this controller good?" becomes a number, not an eyeball.

### Step 6.1 — `StepResponseMetrics`
**Goal:** From `(time, output)` and a setpoint, compute steady-state value & error, percent
overshoot, rise time, and settling time.
**Control-theory specifics (definitions + a closed-form oracle).**
- **Steady-state value** `y_ss`: the final output (average of the tail); **error** `= |setpoint
  − y_ss|`.
- **Percent overshoot** `= max(0, (y_peak − y_ss)/y_ss)·100`.
- **Rise time:** time to go from 10% to 90% of `y_ss`.
- **Settling time (2%):** the last time the output is outside `±2%` of `y_ss`.
- **Oracle — a standard second-order system** `G=ωₙ²/(s²+2ζωₙs+ωₙ²)`: closed-loop unit-step
  **overshoot `Mp = exp(−πζ/√(1−ζ²))`** and **peak time `tp = π/(ωₙ√(1−ζ²))`**. For
  `ζ=0.5, ωₙ=1`: `Mp ≈ 0.163` (16.3%), `tp ≈ 3.63 s`.
**Red — write the test:** `tests/control/step_response_metrics_test.cc`:
- `test_overshoot_of_second_order` *(closed-form)*: open-loop step of `G=1/(s²+s+1)`
  (`ζ=0.5, ωₙ=1`); computed overshoot ≈ `16.3%`, `0.5%` absolute. (Sabotage anchor.)
- `test_peak_time` *(closed-form)*: peak occurs at `t ≈ 3.63 s`, within one grid step.
- `test_settling_and_steady_state` *(closed-form)*: `y_ss ≈ 1`, settling time finite and
  past the peak; first-order `G=1/(s+1)` has **no** overshoot (0%).
- `test_steady_state_error` *(closed-form)*: ties to a Phase-5 P-only run (`e_ss ≈ 0.1`).
- Build → **red**.
**Green — make it pass:** `control/step_response_metrics.{h,cc}`.
**Why it matters:** These four numbers are how every controller is judged; computing them from
your own samples (and checking against the analytic `Mp`) closes the loop between simulation
and control theory.
**Gate:** the metrics tests pass. Faithfulness: report raw `y_peak` as the overshoot (forget
to subtract `y_ss` and normalize) → `test_overshoot_of_second_order` goes red.
**Commit:** `feat(control): add step-response metrics (overshoot, rise, settling)`

---

## Phase 7 — The application

Turn the library into the program the user wants: *type in a transfer function, see the PID
response.*

### Step 7.1 — `main.cc`: TF → PID → plot
**Goal:** Replace the placeholder [`app/main.cc`](app/main.cc) with the real demo: define a
`TransferFunction`, realize it, close a PID loop, simulate, and **plot** setpoint vs. output
(and the control signal) with matplotlib-cpp.
**Control-theory specifics:** reuse the whole stack — `linspace` for the grid,
`toStateSpace()` for the plant, `PIDController` + `ClosedLoopSimulator` for the run,
`StepResponseMetrics` printed to stdout. Plot `output` and a dashed `setpoint` line vs `time`,
plus `control` on a second axis/figure (mirrors the notebook's final plot).
**Red — write the test:** `tests/control/app_smoke_test.cc` (link the `control` library, not
the GUI):
- `test_demo_pipeline_runs` *(smoke)*: the non-plotting core of `main` — build a TF, simulate
  a PID loop, compute metrics — returns finite results with the output length equal to the
  grid. (Factor the pipeline into a testable `runDemo()` so the test doesn't open a window.)
- Build → **red**.
**Green — make it pass:** implement `runDemo()` in the `control` library and have `main.cc`
call it then plot. Update [`app/CMakeLists.txt`](app/CMakeLists.txt) to link `control`.
**Why it matters:** This is the artifact — the moment "PID from scratch" becomes a program you
run. Keeping the pipeline testable (separate from plotting) means the demo can't silently rot.
**Gate:** `test_demo_pipeline_runs` passes and `MainExe` produces a plot. Faithfulness: feed
`runDemo()` an unstable plant (RHP pole) and assert the output diverges — confirming the demo
reflects the real dynamics, not a hard-coded curve.
**Commit:** `feat(app): wire TF→PID→plot demo in main`

---

## Phase 8 — Validation (the real correctness proof)

Unit tests proved each formula matches the theory; these prove the **assembled simulator**
reproduces a known run and obeys the textbook end-to-end.

### Step 8.1 — Reproduce the notebook's thermal-plant PID run
**Goal:** Match [`python/pid.ipynb`](python/pid.ipynb)'s closed-loop result with the same
plant, gains, and setpoint.
**Control-theory specifics:** the notebook's plant is `dT/dt = (1/(τ(1+ε)))(T_f−T) +
(Q/(1+ε))(T_q−T)` with `ε=1, τ=4, T_f=300, Q=2`, controlled to `setpoint=310` by
`PID(Kp=0.6, Ki=0.2, Kd=0.1)`, `Tq` the manipulated variable, `offset=320`. Express this
first-order plant as a `TransferFunction` (or a custom `StateSpace`), run your simulator, and
compare the trajectory to values exported from the notebook.
**Red — write the test:** `tests/control/notebook_golden_test.cc`:
- `test_matches_notebook_trajectory` *(golden)*: export the notebook's `y_sol` (e.g. to a
  small CSV/header committed under `tests/control/data/`), run your simulator on the same grid
  and gains, and assert agreement to `1e-2` (loose, since the notebook uses scipy `odeint`
  while you use RK4). Run → **red** until the data file and mapping exist.
**Green — make it pass:** add the exported reference data; reconcile any constant-offset or
indexing differences (this is where the notebook's `offset=320` and first-step `Tₛ` guard
matter).
**Why it matters:** The end-to-end "does it reproduce a run I already trust?" check — the honest
system-level verdict that your from-scratch stack equals the Python prototype.
**Gate:** `test_matches_notebook_trajectory` passes. Faithfulness: shift the setpoint by 5
units → the test goes red (the bar tracks the real trajectory).
**Commit:** `test(validation): reproduce notebook thermal-plant PID run`

### Step 8.2 — Second-order closed-loop matches analytic metrics
**Goal:** A genuinely second-order plant under PID hits the analytic overshoot/settling within
tolerance.
**Red — write the test:** `tests/control/second_order_validation_test.cc`:
- `test_closed_loop_overshoot_tracks_theory` *(closed-form)*: tune a known `ζ, ωₙ` closed-loop
  (e.g. via the resulting characteristic polynomial), simulate, and assert the measured
  overshoot/peak-time match `Mp`/`tp` within a few percent.
- `test_higher_kd_reduces_overshoot` *(behavioural)*: increasing `Kd` monotonically lowers the
  measured overshoot on a fixed plant.
- Run → **red**.
**Green — make it pass:** validation only — fix whatever the assertions expose.
**Why it matters:** Confirms the loop is right not just at steady state (Phase 5) but in its
*transient* shape — the part PID tuning actually targets.
**Gate:** both tests pass. Faithfulness: a random-gain run must *fail* the overshoot threshold,
so the bar means something.
**Commit:** `test(validation): second-order overshoot/settling vs analytic`

---

## Phase 9 — Polish (optional but recommended)

### Step 9.1 — Transfer-function input from the command line / config
**Goal:** Let the user supply `numerator`, `denominator`, gains, setpoint, and horizon without
recompiling.
**Red — write the test:** `tests/control/cli_test.cc::test_parses_tf_and_gains` *(contract)*:
parsing `"1,3,2"` → `den=[1,3,2]`, etc.; malformed input is rejected with a clear error.
**Green — make it pass:** a small parser (`argv` or a config file) feeding `runDemo()`.
**Gate:** parsing tests pass; `MainExe "5" "2,1" --kp 2 --ki 0.5` runs. Faithfulness: feed a
denominator of lower degree than the numerator → rejected as improper.
**Commit:** `feat(app): accept transfer function + gains as input`

### Step 9.2 — README + reproduce instructions
**Goal:** One page that builds, tests, and reproduces a Phase-8 plot on a clean checkout.
**Red — write the test:** the suite *is* the test — `ctest` green from a fresh `build/`, and a
README "Reproduce" section whose commands you actually run.
**Green — make it pass:** fill in [`README.md`](README.md) (it currently trails off): the
TF→PID→plot workflow, the build/test commands (Linux *and* the existing Windows notes), and a
sample output figure. Clean up the placeholder text.
**Gate:** a clean-checkout `cmake … && ctest` is green and the README commands work.
**Commit:** `docs(readme): document build, test, and reproduce workflow`

### Step 9.3 — A small gallery of example plants
**Goal:** Ship a few ready-to-run plants (integrator `1/s`, first-order, under/over-damped
second-order) so the demo shows the controller across regimes.
**Red — write the test:** `test_gallery_plants_are_stable_closed_loop` *(behavioural)*: each
example, with its shipped gains, settles to setpoint within the horizon.
**Green — make it pass:** add the examples (a header table or small functions) and reference
them from the README.
**Gate:** the gallery test passes. Faithfulness: a deliberately under-damped gain set must
breach an overshoot bound, proving the test discriminates.
**Commit:** `docs(examples): add example-plant gallery`

---

## Key formulas & defaults (one place to look them up)

| Topic | Formula / value |
|---|---|
| **Coeff convention** | descending powers; constant term last; `num=[b_m…b_0]`, `den=[a_n…a_0]` |
| **linspace** | `n` points, spacing `(stop−start)/(n−1)`, both endpoints included |
| **RK4 step** | `k₁=f(t,x)`, `k₂=f(t+h/2,x+h k₁/2)`, `k₃=f(t+h/2,x+h k₂/2)`, `k₄=f(t+h,x+h k₃)`, `x+=(h/6)(k₁+2k₂+2k₃+k₄)`; global error `O(h⁴)` |
| **DC gain** | `G(0)=b₀/a₀`; unit-step final value; `= −CA⁻¹B + D` |
| **Poles / zeros** | roots of denominator / numerator (companion-matrix eigenvalues) |
| **Controllable form** | companion `A` (bottom row `−aᵢ`), `B=[0…0 1]ᵀ`, `C=[b₀…b_{n−1}]`, `D=0`; `eig(A)=poles` |
| **First-order step** | `G=K/(τs+1)` ⇒ `y(t)=K(1−e^(−t/τ))`, `y(τ)=0.632K` |
| **Second-order** | `Mp=exp(−πζ/√(1−ζ²))`, `tp=π/(ωₙ√(1−ζ²))`, `t_s(2%)≈4/(ζωₙ)` |
| **PID (discrete)** | `P=Kp·e`; `I[k]=I[k−1]+Ki(Tₛ/2)(e[k]+e[k−1])`; `D=Kd(e[k]−e[k−1])/Tₛ`; `u=offset+P+I+D` |
| **P-only offset** | first-order unity feedback: `e_∞=1/(1+KpK)` |
| **Integral action** | `Ki>0 ⇒ e_∞→0` |
| **Notebook plant** | `ε=1, τ=4, T_f=300, Q=2`; `PID(0.6,0.2,0.1)`, setpoint 310, offset 320 |
| **Tolerances** | exact algebra `1e-9`; analytic `1e-6`; RK4 trajectory `1e-4`; convergence ratio ≈16 ±10% |

---

## Control concept → C++ component map

| Concept | C++ home |
|---|---|
| Time grid | `math_operation/linspace` |
| Polynomial eval / roots | `math_operation/polynomial` |
| ODE integration (RK4) | `math_operation/ode` (`ODESolver::rk4Step`) |
| `G(s)` = num/den, DC gain, poles, zeros | `control/transfer_function` |
| `(A,B,C,D)` + `derivative`/`output` | `control/state_space` |
| `G(s) → (A,B,C,D)` (controllable form) | `control/transfer_function::toStateSpace` |
| Open-loop step simulation | `control/state_space` (`simulateStep`) |
| Discretized PID + anti-windup | `control/pid_controller` |
| Feedback loop | `control/closed_loop_simulator` |
| Overshoot / rise / settling | `control/step_response_metrics` |
| TF in → plot out | `app/main.cc` (`runDemo`) |

---

## Suggested directory structure

Tests are first-class: each `*_test.cc` is written **before** the code it checks.

```
pid-from-scratch/
├── app/
│   ├── main.cc                       # TF → PID closed loop → plot
│   └── CMakeLists.txt                # links control + matplotlib
├── src/
│   ├── math_operation/
│   │   ├── include/{factorial,ode,linspace,polynomial}.h
│   │   ├── {factorial,ode,linspace,polynomial}.cc
│   │   └── CMakeLists.txt
│   ├── control/
│   │   ├── include/{transfer_function,state_space,pid_controller,
│   │   │            closed_loop_simulator,step_response_metrics}.h
│   │   ├── {…}.cc
│   │   └── CMakeLists.txt            # new STATIC lib, links Eigen
│   └── external_lib/                 # eigen-lib-3.4.0, matplotlib (unchanged)
├── tests/
│   ├── CMakeLists.txt                 # add_subdirectory(math_operation) + (control), (support)
│   ├── support/
│   │   ├── eigen_matchers.h           # 0.2 (header-only)
│   │   ├── eigen_matchers_test.cc     # 0.2 (the meta-test)
│   │   └── CMakeLists.txt             # 0.2 (registered via tests/CMakeLists.txt)
│   ├── math_operation/                # CMakeLists.txt gains linspace_test, polynomial_test
│   │   └── {linspace,polynomial,ode}_test.cc
│   └── control/                       # new CMakeLists.txt: one executable per *_test.cc
│       └── {transfer_function,state_space,pid_controller,
│            closed_loop_simulator,step_response_metrics,
│            notebook_golden,second_order_validation}_test.cc
├── doc/
│   ├── plant_description.md          # existing
│   └── adr-001-architecture.md       # 0.3
├── python/pid.ipynb                  # the reference prototype
├── ROADMAP.md
└── README.md
```

---

## Dependencies

**Only two**, both already vendored under `src/external_lib/`: **Eigen 3.4.0** (matrices,
eigensolver) and **matplotlib-cpp** (plotting, which calls Python's matplotlib at runtime).
Build with **CMake ≥ 3.22**; tests use **GoogleTest** (already fetched via `FetchContent`).
No MATLAB, no `python-control` — that's the whole point.

---

## Milestone checklist (tape this above your desk)

Each box flips only when its step's test was written first, seen to fail, is now green — *and
you've sabotaged the code to confirm the test can fail.*

- [ ] **Phase 0** — `ctest` green on bundled Eigen; Eigen matchers meta-tested; ADR written
- [ ] **Phase 1** — linspace endpoints; polynomial roots (companion matrix); **RK4 4th-order convergence**
- [ ] **Phase 2** — TF DC gain; poles == roots(denominator)
- [ ] **Phase 3** — `eig(A) == poles`; first-order step matches `K(1−e^(−t/τ))`
- [ ] **Phase 4** — PID P/I/D hand values; **trapezoidal** integral; anti-windup
- [ ] **Phase 5** — loop settles to setpoint; **P-offset `1/(1+KpK)`**; integral kills offset
- [ ] **Phase 6** — overshoot/peak-time match analytic second-order `Mp`/`tp`
- [ ] **Phase 7** — `MainExe`: type a TF, get a PID plot; pipeline smoke-tested
- [ ] **Phase 8** — reproduces the notebook run; second-order transient matches theory
- [ ] **Phase 9** — CLI input; README reproduce section; example gallery
