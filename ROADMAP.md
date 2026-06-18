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

Every step has the same shape. Read it top to bottom: the prose orients you first, the
formulas then pin down the exact numbers, and finally the test / implementation / gate turn it
green. The single most important part is **"What we're building"** — read it before the
formulas, because the formulas only make sense once you know what they are *for*.

> **Goal** — the one-sentence outcome.
>
> **What we're building (read this first)** — a plain-English orientation: what the component
> *is*, the job it does inside the simulator, and a walk through each moving part in sentences
> — *before* any equation. This is the part that makes the step understandable; everything
> below is detail hung on this frame.
>
> **Control-theory specifics** — the exact formulas/conventions your test must assert, so you
> pin down the *true* answer rather than a guess. By the time you reach this block you already
> know what each symbol *means*, so it reads as "here are the exact values," not a cold wall
> of symbols.
>
> **Red — write the test** — the test file, the interface it pins down, the oracle type, the
> assertions and tolerance, and a **worked example** (tiny concrete inputs and the value you
> expect back). End of Red = a test that **fails for the right reason** (usually the
> class/function doesn't exist, so it won't compile).
>
> **Green — make it pass** — the implementation to write, plus the one subtlety most likely to
> bite and the **§Wiring** recipe to register it in the build.
>
> **Why it matters** — the transferable engineering / control skill.
>
> **Gate** — the command that must pass, plus the **faithfulness sabotage**: a change you make
> to your own code to confirm the test goes red.
>
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

**What we're building (read this first).** Before writing any control code, you need one
command that builds everything and runs every test, and you need it to come back **green** on
the code that already exists. This is not busywork: TDD only works if "all green" is a state
you can trust and return to. If your baseline is broken or your test runner is silently finding
*zero* tests, then later, when you write a real test and it passes, you won't know whether it
passed because your code is right or because the runner isn't actually running it. So this step
is about making the harness honest: build the project, discover the two existing tests
(`factorial_test`, `ode_test`), and watch them pass — from a single command you'll reuse at
every step after this.

**Control-theory specifics:** none — this is pure build/test plumbing.

**Red — write the test:** there is no new test to write; the "test" is *running the existing
suite*. From the **project root** (the directory containing `CMakeLists.txt`), targeting a
fresh `build/` subdirectory, run:
```
cmake -S . -B build && cmake --build build && ctest --test-dir build
```
**The worked example of "red" here:** if the build succeeds but `ctest` prints
**`No tests were found!!!`**, that is your red. The cause is a missing `enable_testing()` in the
**root** [`CMakeLists.txt`](CMakeLists.txt) — it was present only in
[`tests/math_operation/CMakeLists.txt`](tests/math_operation/CMakeLists.txt), and `ctest` does
not descend into a subdirectory's tests unless testing is enabled at the top level first.

**Green — make it pass:** add `enable_testing()` near the top of the root `CMakeLists.txt`
(before the `add_subdirectory(...)` calls), re-run the command, and confirm `ctest` now lists
and passes `factorial_test` and `ode_test`. Also confirm the bundled Eigen path compiles
(`app/main.cc` includes `Eigen/Dense`).

**Why it matters:** TDD is impossible without a trustworthy "all green" you can return to. A
runner that finds no tests but exits 0 is worse than a failing one — it lies.

**Gate:** `ctest --test-dir build` reports all tests passed and *names* them.
**Faithfulness:** temporarily break one `EXPECT_EQ` in `factorial_test.cc` → `ctest` goes red →
revert. (If breaking a test does *not* turn the bar red, your runner isn't really running it —
fix that before going further.)

**Commit:** `chore(build): verify ctest pipeline green on bundled Eigen`

### Step 0.2 — Eigen test matchers (built test-first)

**Goal:** Build the comparison helpers *every later test* will call, so you assert "these two
vectors / matrices / pole-sets are close" in one line instead of looping by hand.

**What we're building (read this first).** Almost every test in this roadmap compares Eigen
objects: "does this computed `VectorXd` match the analytic one?", "are these poles the set
`{−1, −2}`?". GoogleTest's `ASSERT_NEAR` only knows how to compare two scalars, so without help
you'd hand-roll a loop in every test. You write three reusable matchers now, once, and
**meta-test them** — i.e. write tests *of the matchers themselves* — because a comparison tool
you don't trust is worse than none: if your matcher always returns "pass," every later green bar
is a lie. The three helpers return `::testing::AssertionResult` (not `bool`) so that when they
fail, the message tells you *where* and *by how much*:

- **`expectVectorNear(a, b, tol)`** — checks two `Eigen::VectorXd` agree element-by-element
  (`|aᵢ − bᵢ| ≤ tol`); on failure it reports the worst index and its error.
- **`expectMatrixNear(A, B, tol)`** — the same idea for `Eigen::MatrixXd`.
- **`expectComplexSetNear(a, b, tol)`** — compares two `Eigen::VectorXcd` **as unordered sets**.
  This matters because an eigensolver returns poles in whatever order it likes, so `{−1, −2}`
  and `{−2, −1}` must count as equal. It sorts both by `(real, imag)` and then compares.

**Control-theory specifics:** none yet — but `expectComplexSetNear`'s order-insensitivity is
what later lets a poles test assert `{−1, −2}` without caring which one the solver returns first.

**Red — write the test:** `tests/support/eigen_matchers_test.cc`, with **meta-tests** that each
matcher passes when it should and fails when it should:
- `test_vector_near_passes_and_fails`: **worked example** — let `v = [1, 2, 3]`. Then
  `expectVectorNear(v, v, 1e-9)` must pass; and `expectVectorNear(v, [1, 2, 4], 1e-9)` must
  fail, with a message naming index `2` and error `1.0`.
- `test_complex_set_is_order_insensitive`: **worked example** — `{1+0i, 2+0i}` vs `{2+0i, 1+0i}`
  passes; `{1, 2}` vs `{1, 3}` fails.
- Build → **red** (`eigen_matchers.h` doesn't exist, so the test won't compile).

**Green — make it pass:** write `tests/support/eigen_matchers.h` (header-only) with the three
helpers. **Wire it** per **§Wiring D** (the meta-test includes `Eigen/Dense` directly, so it
needs the Eigen path on its own target) and **§Wiring E.5** (create `tests/support/CMakeLists.txt`,
register it via `add_subdirectory(support)` in [`tests/CMakeLists.txt`](tests/CMakeLists.txt),
and put `tests/support` on the include path so test files can `#include "eigen_matchers.h"`).

**Why it matters:** Your oracles are only as trustworthy as the tools that apply them;
meta-testing them first means every later "green" actually means something.

**Gate:** the matcher meta-tests pass. **Faithfulness:** make `expectVectorNear` `return
::testing::AssertionSuccess()` unconditionally → the failure half of its meta-test goes red,
proving the meta-test bites.

**Commit:** `test(support): add Eigen vector/matrix/complex-set matchers`

### Step 0.3 — Architecture decision record (design checkpoint)

**Goal:** Decide, once and in writing, how the pieces map to C++ types so Phases 1–6 become
mechanical "fill in the class" work instead of re-litigating the same questions mid-stream.

**What we're building (read this first).** This step produces a *document*, not code. An **ADR**
(Architecture Decision Record) is a short markdown file that captures one set of design
decisions — the context, the choice, and the reasoning — so that weeks later nobody (including
you) has to reverse-engineer "why is it built this way?". Here you write **ADR-001**, fixing the
handful of conventions every later module leans on: how polynomial coefficients are ordered,
which library each type lives in, how a state-space system is represented, which TF→state-space
realization you use, and how the PID controller holds its state. Settling these now is what lets
a later step say "implement `TransferFunction::toStateSpace()`" without you having to re-decide
the matrix layout on the spot.

**Decisions to record (in `doc/adr-001-architecture.md`):**
- **Coefficient convention:** descending powers, constant term last (as in the conventions above).
- **Module split:** numerical primitives (`linspace`, polynomial, RK4) live in the existing
  **`math_operation`** library; control types (`TransferFunction`, `StateSpace`,
  `PIDController`, `ClosedLoopSimulator`, `StepResponseMetrics`) live in a **new `control`**
  library under `src/control/` mirroring the `math_operation` layout.
- **State-space representation:** a `StateSpace` holding `Eigen::MatrixXd A, B, C, D`, with
  `derivative(x,u) = A·x + B·u` and `output(x,u) = C·x + D·u`.
- **TF→SS realization:** **controllable canonical form** (companion `A`). Write the exact matrix
  layout into the ADR as a falsifiable example (the 2×2 case from Step 3.2).
- **PID state:** the controller is *stateful* (holds `integral`, `e_prev`, `time_prev`) and
  exposes `reset()`; `compute(setpoint, measurement, time)` mirrors the notebook's signature.
- **Scope:** SISO, continuous-time, strictly-proper-first (support proper `D≠0` later).

**Checkpoint (not a unit test):** the ADR exists and its canonical-form example is concrete
enough to **copy directly into Step 3.2's test** (exact `A`, `B`, `C`, `D` for a named `G(s)`).

**Why it matters:** This is real software architecture. One good decision recorded here turns
Phases 1–6 into a sequence of small unambiguous wins instead of a running argument with yourself.

**Gate:** ADR written; the example is concrete enough to paste into a later test.

**Commit:** `docs(adr): record coefficient, module, and state-space conventions`

---

## Phase 1 — Numerical primitives

Pure functions in the `math_operation` library — the easiest Red/Green wins, and the foundation
the plant simulation stands on.

### Step 1.1 — `linspace`

**Goal:** The time-grid generator — the C++ analog of the notebook's `np.linspace`.

**What we're building (read this first).** Every simulation in this project runs on a grid of
time points: `t = 0, 0.1, 0.2, …`. `linspace(start, stop, n)` builds that grid — `n` points
evenly spaced from `start` to `stop`, **including both endpoints**. The one thing that bites
people is the spacing: with `n` points there are `n − 1` *gaps* between them, so the step is
`(stop − start)/(n − 1)`, not `/n`. Get that wrong and the last point lands slightly short of
`stop`, which silently shifts every later sample and quietly corrupts every plot. So this trivial
function is worth pinning down precisely with a test.

**Control-theory specifics:** `linspace(start, stop, n)` returns `n` points, equally spaced by
`h = (stop − start)/(n − 1)`, with `[0] == start` and `[n−1] == stop` exactly. The edge case
`n == 1` returns `{start}`.

**Red — write the test:** `tests/math_operation/linspace_test.cc`:
- `test_endpoints_and_count` *(closed-form)*: **worked example** — `linspace(0, 10, 50)` has
  size `50`, `[0] == 0`, and `[49] == 10`.
- `test_uniform_spacing` *(closed-form)*: every consecutive difference equals `10/49`
  (tolerance `1e-12`).
- `test_single_point` *(edge)*: `linspace(3, 7, 1)` is `{3}`.
- Build → **red** (`linspace` doesn't exist).

**Green — make it pass:** write `math_operation/linspace.{h,cc}` returning `Eigen::VectorXd`.
**Wire it** per **§Wiring A + B**, and — because `linspace.h` is the **first** `math_operation`
header to include `Eigen/Dense` — apply **§Wiring C** now (make Eigen a `PUBLIC` include of
`MATH_OPERATION`), or `linspace_test` will fail with `Eigen/Dense: No such file or directory`.

**Why it matters:** Every simulation runs on a time grid; an off-by-one on the endpoint shifts
every later sample without ever throwing an error.

**Gate:** the linspace tests pass. **Faithfulness:** divide by `n` instead of `n − 1` → the
spacing test (and `[49] == 10`) go red.

**Commit:** `feat(math): add linspace time-grid generator`

### Step 1.2 — Polynomial evaluation and roots

**Goal:** Evaluate a polynomial and find its roots — the kernel behind DC gain, poles, and zeros.

**What we're building (read this first).** A transfer function is two polynomials (numerator and
denominator), and almost every question you ask of it reduces to one of two operations on those
polynomials: *evaluate* it at a point, or *find its roots*. So you build both now, in the
`math_operation` library, and reuse them everywhere later.

- **Evaluation** uses **Horner's method**: instead of computing each power of `x` separately,
  you nest the multiplications — `p(x) = ((c₀·x + c₁)·x + c₂)·x + …`. It's fewer operations and
  more numerically stable. You provide a real overload and a `std::complex<double>` overload,
  because you'll need to evaluate a polynomial at a complex root.
- **Roots** uses the **companion-matrix trick**: there is a special matrix whose characteristic
  polynomial is exactly your polynomial, so its **eigenvalues are exactly the roots**. You first
  normalize the polynomial to *monic* (divide through by the leading coefficient so the highest
  power has coefficient 1), build that matrix, and hand it to `Eigen::EigenSolver`. This is the
  bridge from "I have coefficients" to "I have roots," and it's the same companion structure
  you'll reuse for the controllable canonical form in Step 3.2 — so building it here pays off
  twice.

**Control-theory specifics:**
- **Eval (Horner, descending coeffs):** `p(x) = ((c₀·x + c₁)·x + c₂)…`.
- **Roots via the companion matrix:** normalize to monic (divide by the leading coefficient);
  build the `n×n` companion matrix of `sⁿ + d_{n−1}sⁿ⁻¹ + … + d₀`, whose eigenvalues are the
  roots; use `Eigen::EigenSolver<Eigen::MatrixXd>` and return `.eigenvalues()` as
  `Eigen::VectorXcd`.

**Red — write the test:** `tests/math_operation/polynomial_test.cc`:
- `test_eval_hand_values` *(closed-form)*: **worked example** — the polynomial `[1, −3, 2]`
  (that is, `s² − 3s + 2`) evaluates to `2` at `s = 0`, `0` at `s = 1`, and `0` at `s = 2`
  (tolerance `1e-9`).
- `test_real_roots` *(closed-form)*: `roots([1, −3, 2]) == {1, 2}` (use `expectComplexSetNear`,
  `1e-9`).
- `test_complex_roots` *(closed-form)*: `roots([1, 0, 1])` (that is, `s² + 1`) `== {+i, −i}`.
- `test_roots_match_eval` *(invariant)*: each returned root `r` satisfies `|p(r)| < 1e-8`.
- Build → **red**.

**Green — make it pass:** write `math_operation/polynomial.{h,cc}` (`polynomialEval`,
`polynomialRoots`). Wire it per **§Wiring A + B**.

**Why it matters:** Poles, zeros, and DC gain all reduce to these two operations; the
companion-matrix trick is the bridge from algebra to Eigen's eigensolver.

**Gate:** the polynomial tests pass. **Faithfulness:** drop the monic normalization (skip
dividing by the leading coefficient) → `test_real_roots` goes red on a non-monic input like
`[2, −6, 4]` (same roots `{1, 2}`, but the un-normalized companion matrix gives the wrong
eigenvalues).

**Commit:** `feat(math): add polynomial eval + roots (companion matrix)`

### Step 1.3 — Vector-valued RK4 integrator

**Goal:** One classic 4-stage Runge–Kutta step for a **state-vector** ODE `ẋ = f(t, x)`,
generalizing the existing scalar Ralston RK2 so it can integrate `ẋ = A·x + B·u`.

**What we're building (read this first).** This is the engine that actually advances the plant
through time. Your plant's dynamics are an ODE `ẋ = f(t, x)` where the state `x` is a *vector*
(an `n`-dimensional state for an `n`-th-order system), so the existing scalar Ralston solver
isn't enough — you need one that steps a whole vector forward. **RK4** (the classic
"fourth-order Runge–Kutta") does this by sampling the slope `f` at four places within the step
and taking a weighted average: once at the start (`k₁`), twice at the midpoint (`k₂`, `k₃`), and
once at the end (`k₄`), combined as `(k₁ + 2k₂ + 2k₃ + k₄)/6`. That particular weighting is what
makes the method *fourth-order accurate*: halve the step size and the error drops by ~16×
(`2⁴`). A nice consequence you can test exactly: for a slope that depends only on time, RK4
reduces to **Simpson's rule** and integrates any cubic with **zero error**.

**Control-theory specifics (classic RK4 — assert these exact weights):**
```
k₁ = f(t,        x)
k₂ = f(t + h/2,  x + (h/2)·k₁)
k₃ = f(t + h/2,  x + (h/2)·k₂)
k₄ = f(t + h,    x + h·k₃)
x_next = x + (h/6)·(k₁ + 2k₂ + 2k₃ + k₄)
```
RK4 integrates `ẋ = f(t)` **exactly for polynomials of degree ≤ 3** (it reduces to Simpson's
rule), and its **global** error is `O(h⁴)`.

**Red — write the test:** add to `tests/math_operation/ode_test.cc` (keep the existing Ralston
test):
- `test_rk4_exact_on_cubic` *(closed-form)*: **worked example** — for `ẋ = 3t²` with `x(0) = 0`
  (whose exact solution is `x = t³`), one step of size `h = 0.5` gives **exactly** `0.125`
  (tolerance `1e-12`), because RK4 is Simpson-exact for cubics.
- `test_rk4_matches_exponential` *(closed-form)*: `ẋ = x`, `x(0) = 1`, integrated to `t = 1`
  with small `h`, gives `≈ e` (tolerance `1e-4`).
- `test_rk4_fourth_order_convergence` *(convergence — the point)*: integrate `ẋ = x` to a fixed
  `t` at step `h` and again at `h/2`; the ratio of the two errors is `≈ 16` (within 10%).
- `test_rk4_vector_state` *(contract)*: a 2-D linear system `ẋ = A·x` integrates each component
  correctly versus its matrix-exponential value (tolerance `1e-4`).
- Build → **red**.

**Green — make it pass:** extend `ODESolver` in `math_operation/ode.{h,cc}` with
`Eigen::VectorXd rk4Step(const std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>& f,
double t, const Eigen::VectorXd& x, double stepSize)`. The header now needs `#include <functional>`
and `#include "Eigen/Dense"`; this relies on the `MATH_OPERATION` public-Eigen wiring from
**§Wiring C** (done in Step 1.1) — without it, the existing `ode_test` will stop compiling.

**Why it matters:** This is the engine that advances the plant. The convergence test is what
*proves* it's genuinely fourth-order rather than merely "looks about right."

**Gate:** the RK4 tests pass. **Faithfulness:** change the weights `(1, 2, 2, 1)/6` to
`(1, 1, 1, 1)/4` → `test_rk4_exact_on_cubic` and the convergence test go red.

**Commit:** `feat(math): add vector RK4 integrator (ODESolver::rk4Step)`

---

## Phase 2 — Transfer functions

The user-facing input. A `TransferFunction` stores numerator/denominator coefficients and
answers the questions control engineers ask of `G(s)`.

### Step 2.1 — `TransferFunction` construction, DC gain, properness

**Goal:** Build `G(s) = num(s)/den(s)` and read off its DC (steady-state) gain.

**What we're building (read this first).** This is the object the *user* types in: a transfer
function `G(s)`, stored as its numerator and denominator coefficients. Beyond just holding the
data, it answers two first questions a control engineer always asks:

- **DC gain** — "where does the output finally settle for a constant input?" For a step input,
  the final value is `G(0)`, which is just the ratio of the two constant terms (`b₀/a₀`). This
  is your very first sanity check on any plant: if you command a unit step and the DC gain is
  `5`, the output had better head toward `5`.
- **Properness** — "is this transfer function physically realizable / simulatable?" A transfer
  function is *proper* if its numerator degree doesn't exceed its denominator degree, and
  *strictly proper* if it's strictly less. This decides whether a state-space realization even
  exists, so it's worth a predicate now.

This is also the first code in the **new `control` library**, so standing that library up is
part of this step.

**Control-theory specifics:**
- Store `Eigen::VectorXd numerator, denominator` (descending powers). Reject an empty or all-zero
  denominator.
- **DC gain** `G(0) = b₀/a₀` = (last numerator coeff)/(last denominator coeff). By the
  final-value theorem this is the unit-step **final value**.
- **Proper** iff `deg(num) ≤ deg(den)` (i.e. `numerator.size() ≤ denominator.size()`);
  **strictly proper** iff `<`.

**Red — write the test:** `tests/control/transfer_function_test.cc`:
- `test_dc_gain_first_order` *(closed-form)*: **worked example** — `G = 5/(2s + 1)`, so
  `num = [5]`, `den = [2, 1]`, and `dcGain() == 5/1 == 5`.
- `test_dc_gain_general` *(closed-form)*: `G = (s + 3)/(s² + 2s + 6)` → `dcGain() == 3/6 == 0.5`.
- `test_properness` *(contract)*: a strictly-proper case, an equal-degree (proper) case, and an
  improper case each classify correctly.
- `test_rejects_zero_denominator` *(edge)*: constructing with an all-zero denominator throws.
- Build → **red**.

**Green — make it pass:** this is the **first `control` code**, so before it can compile you must
**stand up the new `control` library and `tests/control/` directory per §Wiring E** (new
`src/control/CMakeLists.txt` + `add_subdirectory(control)` in both
[`src/CMakeLists.txt`](src/CMakeLists.txt) and [`tests/CMakeLists.txt`](tests/CMakeLists.txt)).
Then write `control/transfer_function.{h,cc}` (`dcGain`, `isProper`, `isStrictlyProper`).

**Why it matters:** DC gain is the first sanity check on any loop ("where does it settle?");
properness decides whether a state-space realization even exists.

**Gate:** the TF tests pass. **Faithfulness:** compute DC gain from the **leading** coefficients
(`num[0]/den[0]`) instead of the constant terms → `test_dc_gain_first_order` goes red.

**Commit:** `feat(control): add TransferFunction (dc gain, properness)`

### Step 2.2 — Poles and zeros

**Goal:** Expose `poles()` and `zeros()` of `G(s)`.

**What we're building (read this first).** The **poles** of a transfer function *are* its
dynamics: they're the roots of the denominator, and their location in the complex plane tells
you everything about stability and the shape of the response (a pole at `−1` decays like
`e^(−t)`; a complex pair gives oscillation; any pole with a positive real part means the system
blows up). The **zeros** are the roots of the numerator and shape how inputs are transmitted.
You already built a root finder in Step 1.2, so this step is mostly *wiring*: `poles()` is
`polynomialRoots(denominator)`, `zeros()` is `polynomialRoots(numerator)`. You also add a small
`isStable()` predicate — "do all poles have a negative real part?" — because it's the single
most common question asked of a plant.

**Control-theory specifics:** `poles() = polynomialRoots(denominator)`,
`zeros() = polynomialRoots(numerator)` (Step 1.2). Stability is "all poles have strictly negative
real part."

**Red — write the test:** add to `transfer_function_test.cc`:
- `test_poles_of_second_order` *(closed-form)*: **worked example** — `G = 1/(s² + 3s + 2)` has
  poles `{−1, −2}` (use `expectComplexSetNear`, `1e-9`).
- `test_complex_poles` *(closed-form)*: `G = 1/(s² + 2s + 5)` → poles `{−1 ± 2i}`.
- `test_zeros` *(closed-form)*: `G = (s² − 1)/(s² + 2s + 2)` → zeros `{−1, +1}`.
- `test_stability_predicate` *(invariant)*: the second-order stable plant is reported stable; a
  plant with a right-half-plane pole is not.
- Build → **red**.

**Green — make it pass:** add `TransferFunction::poles()`, `zeros()`, and `isStable()`.

**Why it matters:** Poles *are* the dynamics; verifying them against the denominator roots is the
invariant that ties the whole transfer-function representation together.

**Gate:** the poles/zeros tests pass. **Faithfulness:** have `poles()` factor the **numerator** by
mistake → `test_poles_of_second_order` goes red.

**Commit:** `feat(control): add poles, zeros, and stability check`

---

## Phase 3 — State space (the simulatable form)

A transfer function isn't directly integrable; its **state-space realization** is. This phase
converts `G(s)` to `(A, B, C, D)` and simulates its open-loop response.

### Step 3.1 — `StateSpace` type

**Goal:** A container for `(A, B, C, D)` with the two operations a simulator needs.

**What we're building (read this first).** A transfer function `G(s)` describes a system in the
frequency domain, but to *simulate* it you need its time-domain twin: the **state-space form**,
four matrices `(A, B, C, D)` that describe the system as a first-order vector ODE. Concretely
they encode two operations the integrator will call over and over:

- **`derivative(x, u) = A·x + B·u`** — given the current state `x` and input `u`, how is the
  state changing right now? (This is the `f(t, x)` that `rk4Step` integrates.)
- **`output(x, u) = C·x + D·u`** — given the current state and input, what does the sensor read?
  For a SISO system this is a single number.

This step is just the *container* plus those two matrix operations, with dimension checks at
construction so a mis-shaped `B` or `C` fails loudly instead of producing garbage later. The
*conversion from `G(s)`* comes in the next step.

**Control-theory specifics:** `derivative(x, u) = A·x + B·u` (an `n`-vector),
`output(x, u) = C·x + D·u` (a scalar for SISO). Validate dimensions at construction (`A` square
`n×n`, `B` is `n×1`, `C` is `1×n`, `D` is `1×1`).

**Red — write the test:** `tests/control/state_space_test.cc`:
- `test_derivative_and_output` *(closed-form)*: **worked example** — pick a tiny hand-set system,
  e.g. `A = [[0, 1], [−2, −3]]`, `B = [[0], [1]]`, `C = [[1, 0]]`, `D = [[0]]`, and a chosen
  `x = [1, 1]`, `u = 1`; assert `derivative` and `output` equal the values you compute by hand
  (tolerance `1e-12`).
- `test_dimension_validation` *(edge)*: a mismatched `B`/`C`/`D` shape throws at construction.
- Build → **red**.

**Green — make it pass:** write `control/state_space.{h,cc}`.

**Why it matters:** This is the clean boundary the integrator talks to; getting `A·x + B·u` right
(and dimension-checked) prevents a whole class of silent shape bugs.

**Gate:** the state-space tests pass. **Faithfulness:** drop the `+ D·u` term in `output` →
`test_derivative_and_output` goes red on a case with `D ≠ 0`.

**Commit:** `feat(control): add StateSpace (derivative + output)`

### Step 3.2 — Transfer function → controllable canonical form

**Goal:** `TransferFunction::toStateSpace()` producing the controllable canonical realization.

**What we're building (read this first).** This is *the* conversion that makes a typed-in
transfer function simulatable. Given `G(s) = num/den`, you produce the four matrices
`(A, B, C, D)` of an equivalent state-space system. There are many valid realizations; you use
the **controllable canonical form**, in which `A` is a *companion matrix* — an identity-shifted-up
pattern with the (negated, normalized) denominator coefficients along the bottom row. Two reasons
to pick it: it's mechanical to build straight from the coefficients, and it makes the result
**self-checking**. Because `A` is a companion matrix of the denominator, its **eigenvalues are
exactly the poles** you already computed in Step 2.2 — so you can assert
`eigenvalues(A) == G.poles()` and catch a sign error or a transposed row instantly. (First
normalize by dividing both polynomials by the leading denominator coefficient, so the form
assumes a monic denominator.)

**Control-theory specifics (assert this exact layout).** For a strictly proper, monic-normalized
`G(s) = (b_{n−1}sⁿ⁻¹ + … + b₀)/(sⁿ + a_{n−1}sⁿ⁻¹ + … + a₀)` (divide both by the leading
denominator coefficient first):
```
      ┌ 0    1    0  … 0   ┐         ┌0┐
      │ 0    0    1  … 0   │         │0│
  A = │ ⋮              ⋱ ⋮ │ ,   B = │⋮│ ,  C = [b₀  b₁ … b_{n−1}] ,  D = 0
      │ 0    0    0  … 1   │         │0│
      └ −a₀ −a₁ −a₂ … −a_{n−1} ┘     └1┘
```
Two invariants make this self-checking: **`eigenvalues(A) == poles(G)`** and
**`G(0) = −C·A⁻¹·B + D`** (the steady-state gain computed from the realization equals the DC gain
from Step 2.1).

**Red — write the test:** add to `state_space_test.cc`:
- `test_eigenvalues_equal_poles` *(invariant — the crux)*: **worked example** — for
  `G = 1/(s² + 3s + 2)`, `eigenvalues(ss.A) == G.poles() == {−1, −2}` (use `expectComplexSetNear`,
  `1e-9`).
- `test_realization_dc_gain` *(invariant)*: `−C·A⁻¹·B + D == G.dcGain()` (`1e-9`) for both a
  first-order and a second-order `G`.
- `test_first_order_layout` *(closed-form)*: `G = K/(τs + 1)` realizes to `A = [−1/τ]`, `B = [1]`,
  `C = [K/τ]`, `D = [0]`.
- Build → **red**.

**Green — make it pass:** write `TransferFunction::toStateSpace()` — reverse the coefficients to
ascending order, normalize to monic, fill the companion `A`, and set `B`, `C`, `D`. The one
subtlety: the stored convention is *descending* powers (constant last), but the companion layout
above indexes `a₀ … a_{n−1}` *ascending*, so you must reverse before filling — exactly the kind
of thing the ADR's worked example was meant to pin down.

**Why it matters:** This is the conversion that makes a transfer function simulatable; the
eigenvalues-equal-poles invariant is an exceptionally strong, sign-sensitive check.

**Gate:** the realization tests pass. **Faithfulness:** put `+aᵢ` instead of `−aᵢ` in the bottom
row of `A` → `test_eigenvalues_equal_poles` goes red (the poles flip sign).

**Commit:** `feat(control): add TF→state-space (controllable canonical form)`

### Step 3.3 — Open-loop step response via RK4

**Goal:** Simulate `ẋ = A·x + B·u`, `y = C·x + D·u` for a constant input over a time grid.

**What we're building (read this first).** Now you put the last three steps together: take a
state-space plant, hold the input constant, and march it through time with `rk4Step` to produce
the **open-loop step response** — the curve you'd see if you poked the plant with a fixed input
and no controller. "Open-loop" means there's no feedback yet; this is the plant on its own. The
reason to build and test it *before* the controller is isolation: a first-order plant has a
textbook exact response, `y(t) = K(1 − e^(−t/τ))`, that you can check sample-for-sample. Once you
know the open-loop simulator reproduces that curve, any discrepancy you see later in the
*closed*-loop is the controller's doing, not a broken integrator. ("Zero-order hold" just means
the input is held constant across each RK4 step — exactly what a digital controller does between
updates.)

**Control-theory specifics:** zero-order hold — `u` is held constant across each RK4 step
(Step 1.3). For a unit step on `G = K/(τs + 1)`, the exact response is
**`y(t) = K(1 − e^(−t/τ))`** — it passes through `0.632·K` at `t = τ` and settles at `K`.

**Red — write the test:** add to `state_space_test.cc`:
- `test_first_order_step_matches_analytic` *(closed-form — the anchor)*: **worked example** —
  simulate `G = 2/(s + 1)` (so `K = 2`, `τ = 1`) under a unit step on a fine grid; every sample
  matches `2·(1 − e^(−t))` to `1e-4`. (At `t = 1` that's `2·0.632 = 1.264`; at large `t` it
  approaches `2`.)
- `test_final_value_is_dc_gain` *(invariant)*: the last sample ≈ `G.dcGain()·u` (`1e-3`).
- `test_zero_input_zero_state_stays_zero` *(edge)*: `x₀ = 0`, `u = 0` ⇒ `y ≡ 0`.
- Build → **red**.

**Green — make it pass:** write a `simulateStep(StateSpace, u, timeVector, x0)` helper (a free
function in `control`, or a method on `StateSpace`) that threads `rk4Step` across the grid and
records `y` at each point.

**Why it matters:** Proving the open-loop simulator against the textbook first-order response
means any later closed-loop discrepancy is the *controller's* doing, not the integrator's.

**Gate:** the step-response tests pass. **Faithfulness:** integrate `A·x` only (drop the `B·u`
term) → `test_first_order_step_matches_analytic` goes red (the output stays at 0).

**Commit:** `feat(control): add open-loop state-space step simulation`

---

## Phase 4 — The PID controller

The discretized PID from [`python/pid.ipynb`](python/pid.ipynb) §"Discretized PID Equation,"
ported to a stateful C++ class.

### Step 4.1 — Discretized P, I, D

**Goal:** `PIDController::compute(setpoint, measurement, time)` implementing the notebook's
equation (5).

**What we're building (read this first).** This is the controller itself — the thing that looks
at how far the measurement is from the setpoint and decides what input to send the plant. The
**error** is `e = setpoint − measurement`, and the output is the sum of three terms, each
answering a different question:

- **Proportional** (`P = Kp·e`) — "how big is the error *right now*?" Push harder the further you
  are from target.
- **Integral** (`I`) — "how much error has *accumulated* over time?" This is what eventually
  drives the steady-state error to zero. You accumulate it the way the notebook does, with the
  **trapezoidal rule**: each step adds `Ki·(Tₛ/2)·(e[k] + e[k−1])` (the area of a trapezoid
  between the last two error samples), where `Tₛ` is the elapsed time since the last call.
- **Derivative** (`D = Kd·(e[k] − e[k−1])/Tₛ`) — "how *fast* is the error changing?" This damps
  the response and reduces overshoot, using a backward difference of the error.

The controller is **stateful**: it remembers the running integral, the previous error, and the
previous time between calls, and `reset()` clears them. Two guards matter — on the very first
call there is no previous error (treat `e_prev = 0`), and you must avoid dividing by zero in the
derivative when `Tₛ ≤ 0` (the notebook substitutes a tiny `Tₛ = 1e-5`).

**Control-theory specifics (the exact discretization — match the notebook):** with error
`e[k] = setpoint − measurement` and step `Tₛ = time − time_prev`:
- **Proportional:** `P = Kp·e[k]`.
- **Integral (trapezoidal accumulator):** `I[k] = I[k−1] + Ki·(Tₛ/2)·(e[k] + e[k−1])`.
- **Derivative (backward difference):** `D = Kd·(e[k] − e[k−1])/Tₛ`.
- **Output:** `u[k] = offset + P + I[k] + D` (the notebook's `offset` is an operating-point bias;
  default it to `0`). First call: `e_prev = 0`; guard `Tₛ ≤ 0` with a tiny `Tₛ`. `reset()` clears
  `I`, `e_prev`, `time_prev`.

**Red — write the test:** `tests/control/pid_controller_test.cc`:
- `test_pure_proportional` *(closed-form)*: **worked example** — `Kp = 2`, `Ki = Kd = 0`; one
  `compute(setpoint = 1, measurement = 0, …)` returns `2·1 = 2`.
- `test_first_step_hand_value` *(closed-form)*: with all three gains set and a known `e, Tₛ`, the
  output equals the `P + I + D` you compute by hand (using `e_prev = 0`), to `1e-9`.
- `test_integral_accumulates_trapezoidally` *(closed-form)*: two calls with **constant** `e` and
  fixed `Tₛ`; the accumulator after step 2 equals the hand-rolled trapezoid sum
  `Ki·(Tₛ/2)·e + Ki·Tₛ·e` (to `1e-9`).
- `test_derivative_on_error_change` *(closed-form)*: with `Kp = Ki = 0`, a jump in `e` from `0`
  to `Δ` over `Tₛ` gives `D = Kd·Δ/Tₛ`.
- `test_reset_clears_state` *(invariant)*: after `reset()`, a repeated input sequence reproduces
  the first run exactly.
- Build → **red**.

**Green — make it pass:** write `control/pid_controller.{h,cc}` (constructor
`(kp, ki, kd, offset = 0)`, plus `compute` and `reset`).

**Why it matters:** This *is* the controller. The trapezoidal/backward-difference choices are
exactly where a "looks plausible" implementation quietly diverges from the math — the hand values
pin them down.

**Gate:** the PID tests pass. **Faithfulness:** replace the trapezoidal integral with a
rectangular one (`Ki·Tₛ·e[k]`) → `test_integral_accumulates_trapezoidally` goes red.

**Commit:** `feat(control): add discretized PID controller (P, I, D)`

### Step 4.2 — Integrator anti-windup (clamping)

**Goal:** Stop the integral from accumulating while the output is saturated.

**What we're building (read this first).** Real actuators have limits — a valve can't open more
than 100%, a motor has a maximum voltage. **Integrator windup** is the classic failure that
follows: while the output is pinned at its limit (saturated), the integral term keeps piling up
error it can't actually act on, and when the error finally reverses, that huge stored integral
makes the controller massively overshoot and oscillate. The standard fix is **conditional
integration**: clamp the output to its `[u_min, u_max]` range, and *whenever the output is
saturated, hold the integral accumulator* — don't add this step's contribution. With no limits
set, the controller behaves exactly as in Step 4.1.

**Control-theory specifics:** given output limits `[u_min, u_max]`, after computing `u`, if it
saturates then **clamp `u`** and **hold the integral accumulator** (do not add this step's
contribution). Without limits, behaviour is identical to Step 4.1.

**Red — write the test:** add to `pid_controller_test.cc`:
- `test_output_clamped` *(closed-form)*: **worked example** — with limits `[0, 10]` and a large
  error that would drive `u` to `50`, the output is exactly `10`.
- `test_no_windup` *(behavioural)*: drive the controller into saturation for many steps, then
  reduce the error; the integral magnitude stays bounded and the response recovers without the
  long overshoot an unclamped integral would produce (compare against the unclamped controller).
- `test_disabled_limits_match_step_4_1` *(invariant)*: with infinite limits, the outputs equal
  the Step 4.1 controller bit-for-bit.
- Build → **red**.

**Green — make it pass:** add `setOutputLimits(min, max)` and conditional integration to
`compute`.

**Why it matters:** Windup is the single most common reason a hand-rolled PID "works in theory,
oscillates in practice"; conditional integration is the standard fix.

**Gate:** the anti-windup tests pass. **Faithfulness:** clamp the output but **still** accumulate
the integral → `test_no_windup` goes red.

**Commit:** `feat(control): add PID output clamping + anti-windup`

---

## Phase 5 — Closed-loop simulation

Wire setpoint → error → PID → plant → measurement → feedback. This is the deliverable: a transfer
function in, a controlled response out.

### Step 5.1 — `ClosedLoopSimulator`

**Goal:** Run the unity-feedback loop over a time grid and return `(time, output, control)`.

**What we're building (read this first).** Here every previous piece snaps together into the
feedback loop that *is* PID control. At each time step the simulator does four things, in order:
**measure** the plant's output, **compute** the control input from the PID controller (which sees
the error between setpoint and measurement), **integrate** the plant forward one step with that
input held constant, and **record** the output and control. Feeding the measurement back in — so
the controller continually corrects itself — is what "closed-loop" means, and it's the whole
point of the project: you hand this thing a `TransferFunction` and a setpoint, and it gives you
back the controlled response over time.

**Control-theory specifics (the loop, matching the notebook's structure).** For each step `k` on
the grid:
1. measure `y[k] = C·x[k] + D·u[k−1]` (with `u[−1]` taken as `0`; for a strictly proper plant
   `D = 0`, so this term vanishes),
2. `u[k] = controller.compute(setpoint, y[k], t[k])`,
3. integrate the plant over `[t[k], t[k+1]]` with `rk4Step`, holding `u[k]` constant (ZOH),
4. record `y`, `u`.

**Red — write the test:** `tests/control/closed_loop_simulator_test.cc`:
- `test_result_shapes` *(contract)*: the returned `time`, `output`, and `control` all have the
  same length as the time grid, and the first recorded output is `C·x₀`.
- `test_settles_near_setpoint_with_pi` *(behavioural)*: **worked example** — a PI controller on a
  stable first-order plant, run long enough, drives `y` to within 1% of the setpoint.
- `test_zero_gains_is_open_loop` *(invariant)*: with `Kp = Ki = Kd = 0`, `u ≡ offset` and the
  output matches the open-loop simulation from Step 3.3.
- Build → **red**.

**Green — make it pass:** write `control/closed_loop_simulator.{h,cc}` with a `SimulationResult`
(`Eigen::VectorXd time, output, control`) returned by
`simulate(plant, controller, setpoint, timeVector, x0)`.

**Why it matters:** This is the first full subsystem; the open-loop-equivalence invariant proves
the loop wiring adds nothing spurious when the controller is silent.

**Gate:** the closed-loop tests pass. **Faithfulness:** compute the error as `y − setpoint`
(positive feedback instead of negative) → `test_settles_near_setpoint_with_pi` goes red (the loop
diverges or settles at the wrong value).

**Commit:** `feat(control): add closed-loop PID simulator`

### Step 5.2 — Steady-state correctness (the controller's signature behaviours)

**Goal:** Prove the two textbook steady-state results that distinguish P from PI control.

**What we're building (read this first).** No new production code here — this step *validates*
the simulator you just built against the two most famous facts in introductory control, turning
control intuition into permanent regression tests:

- **A pure-P controller leaves a permanent offset.** Proportional-only control can never fully
  close the gap: at steady state there's a residual error `e_∞ = 1/(1 + Kp·K)` that never goes
  away. With `K = 1` and `Kp = 9`, the output settles at `0.9` and the error sticks at `0.1`.
- **Adding integral action eliminates that offset.** The integral term keeps accumulating as long
  as *any* error remains, so it drives the steady-state error to zero and the output all the way
  to the setpoint. This is *the* reason PID beats pure-P, and now it's a test.

If these don't pass, your loop math (sign of the feedback, the integral update) is wrong — so
they double as a correctness check on Step 5.1.

**Control-theory specifics (closed-form steady states for a first-order plant `G = K/(τs + 1)`
under unity feedback and a unit-step setpoint):**
- **P-only** (`Kp`, `Ki = Kd = 0`): closed loop `T = Kp·K/(τs + 1 + Kp·K)`, so
  `y_∞ = Kp·K/(1 + Kp·K)` and the **steady-state error is `e_∞ = 1/(1 + Kp·K) ≠ 0`** — a
  permanent offset. With `K = 1`, `Kp = 9`: `y_∞ = 0.9`, `e_∞ = 0.1`.
- **PI** (`Ki > 0`): the integrator forces `e_∞ → 0`, so `y_∞ → setpoint`.

**Red — write the test:** add to `closed_loop_simulator_test.cc`:
- `test_p_only_steady_state_offset` *(closed-form — the anchor)*: **worked example** — `K = 1`,
  `Kp = 9`; after a long-enough run the final output ≈ `0.9` and the steady-state error ≈ `0.1`
  (to `1e-2`).
- `test_integral_eliminates_offset` *(invariant)*: adding `Ki > 0` on the same plant drives the
  final error below `1e-3`.
- Build → **red**.

**Green — make it pass:** no new code if Step 5.1 is correct — these are validation tests of the
existing simulator. Write them, watch them pass; if they don't, the loop math is wrong, so fix
that.

**Why it matters:** "Integral action removes steady-state error" is the defining reason PID beats
pure P; encoding it as a test turns control intuition into a regression guard.

**Gate:** both tests pass. **Faithfulness:** zero out the integral contribution inside `compute`
→ `test_integral_eliminates_offset` goes red.

**Commit:** `test(control): pin P-offset and PI zero-offset steady states`

---

## Phase 6 — Step-response metrics

Quantify a response so "is this controller good?" becomes a number, not an eyeball.

### Step 6.1 — `StepResponseMetrics`

**Goal:** From `(time, output)` and a setpoint, compute steady-state value & error, percent
overshoot, rise time, and settling time.

**What we're building (read this first).** Once you can simulate a response you need to *judge*
it, and engineers judge step responses with four numbers — the same four MATLAB's `stepinfo`
reports. You compute them from your own samples:

- **Steady-state value & error** — where the output finally settles (average of the tail), and
  how far that is from the setpoint.
- **Percent overshoot** — how far the peak shoots *past* the steady-state value, as a percentage.
  This is the headline number for "is it too aggressive?"
- **Rise time** — how long to climb from 10% to 90% of the final value (how *snappy* it is).
- **Settling time** — the last moment the output is still outside a ±2% band around its final
  value (how long until it's "done").

The beautiful part is the oracle: a standard second-order system has **closed-form** overshoot
and peak time in terms of its damping `ζ` and natural frequency `ωₙ`, so you can check your
sample-based numbers against exact theory rather than against another tool.

**Control-theory specifics (definitions + a closed-form oracle).**
- **Steady-state value** `y_ss`: the final output (average of the tail); **error**
  `= |setpoint − y_ss|`.
- **Percent overshoot** `= max(0, (y_peak − y_ss)/y_ss)·100`.
- **Rise time:** time to go from 10% to 90% of `y_ss`.
- **Settling time (2%):** the last time the output is outside `±2%` of `y_ss`.
- **Oracle — a standard second-order system** `G = ωₙ²/(s² + 2ζωₙs + ωₙ²)`: a unit step gives
  **overshoot `Mp = exp(−πζ/√(1 − ζ²))`** and **peak time `tp = π/(ωₙ√(1 − ζ²))`**. For
  `ζ = 0.5, ωₙ = 1`: `Mp ≈ 0.163` (16.3%), `tp ≈ 3.63 s`.

**Red — write the test:** `tests/control/step_response_metrics_test.cc`:
- `test_overshoot_of_second_order` *(closed-form — the anchor)*: **worked example** — the step
  response of `G = 1/(s² + s + 1)` (which is `ζ = 0.5, ωₙ = 1`) has a computed overshoot of
  ≈ `16.3%` (to `0.5%` absolute).
- `test_peak_time` *(closed-form)*: the peak occurs at `t ≈ 3.63 s` (within one grid step).
- `test_settling_and_steady_state` *(closed-form)*: `y_ss ≈ 1`; settling time is finite and past
  the peak; and a first-order `G = 1/(s + 1)` has **no** overshoot (0%).
- `test_steady_state_error` *(closed-form)*: ties to a Phase-5 P-only run (`e_ss ≈ 0.1`).
- Build → **red**.

**Green — make it pass:** write `control/step_response_metrics.{h,cc}`.

**Why it matters:** These four numbers are how every controller is judged; computing them from
your own samples (and checking against the analytic `Mp`) closes the loop between simulation and
control theory.

**Gate:** the metrics tests pass. **Faithfulness:** report the raw `y_peak` as the overshoot
(forgetting to subtract `y_ss` and normalize) → `test_overshoot_of_second_order` goes red.

**Commit:** `feat(control): add step-response metrics (overshoot, rise, settling)`

---

## Phase 7 — The application

Turn the library into the program you actually want: *type in a transfer function, see the PID
response.*

### Step 7.1 — `main.cc`: TF → PID → plot

**Goal:** Replace the placeholder [`app/main.cc`](app/main.cc) with the real demo: define a
`TransferFunction`, realize it, close a PID loop, simulate, and **plot** the result.

**What we're building (read this first).** This is the payoff — the moment "PID from scratch"
becomes a program you run. `main.cc` exercises the entire stack you built: `linspace` for the
time grid, `toStateSpace()` for the plant, `PIDController` + `ClosedLoopSimulator` for the run,
`StepResponseMetrics` printed to the console, and finally a matplotlib-cpp plot of setpoint vs.
output (and the control signal), mirroring the notebook's final figure. The one design rule:
**separate the computation from the plotting**. Factor the numerical pipeline into a `runDemo()`
function that returns data (no GUI), so a test can call it headlessly; `main()` just calls
`runDemo()` and then draws. Otherwise the only way to "test" the app is to open a window and
squint, and the demo silently rots.

**Control-theory specifics:** reuse the whole stack — `linspace` (grid), `toStateSpace()`
(plant), `PIDController` + `ClosedLoopSimulator` (run), `StepResponseMetrics` (printed). Plot
`output` with a dashed `setpoint` line vs `time`, plus `control` on a second axis/figure.

**Red — write the test:** `tests/control/app_smoke_test.cc` (links the `control` library, not the
GUI):
- `test_demo_pipeline_runs` *(smoke)*: **worked example** — call `runDemo()` on a simple stable
  plant and assert it returns finite results whose `output` length equals the time-grid length.
  (Factoring the pipeline into `runDemo()` is what lets this test run without opening a window.)
- Build → **red**.

**Green — make it pass:** implement `runDemo()` in the `control` library, have `main.cc` call it
and then plot, and update [`app/CMakeLists.txt`](app/CMakeLists.txt) to link `control`.

**Why it matters:** This is the artifact. Keeping the pipeline testable (separate from plotting)
means the demo can't silently rot between releases.

**Gate:** `test_demo_pipeline_runs` passes and `MainExe` produces a plot. **Faithfulness:** feed
`runDemo()` an unstable plant (a right-half-plane pole) and assert the output diverges —
confirming the demo reflects the real dynamics, not a hard-coded curve.

**Commit:** `feat(app): wire TF→PID→plot demo in main`

---

## Phase 8 — Validation (the real correctness proof)

Unit tests proved each formula matches the theory; these prove the **assembled simulator**
reproduces a known run and obeys the textbook end-to-end.

### Step 8.1 — Reproduce the notebook's thermal-plant PID run

**Goal:** Match [`python/pid.ipynb`](python/pid.ipynb)'s closed-loop result with the same plant,
gains, and setpoint.

**What we're building (read this first).** Your unit tests each proved "this one formula matches
the theory." This step asks the bigger question: does the *whole assembled simulator* reproduce a
run you already trust? You take the exact thermal-plant scenario from the notebook — same plant
constants, same PID gains, same setpoint and offset — run it through your from-scratch C++ stack,
and compare the resulting trajectory against numbers exported from the notebook. This is a
**golden test**: the reference data is a committed file, and your job is to land on it. The
tolerance is deliberately loose (`1e-2`) because the notebook integrates with scipy's `odeint`
while you use RK4, so the two won't agree to the bit — but they must agree to the eye.

**Control-theory specifics:** the notebook's plant is
`dT/dt = (1/(τ(1+ε)))·(T_f − T) + (Q/(1+ε))·(T_q − T)` with `ε = 1, τ = 4, T_f = 300, Q = 2`,
controlled to `setpoint = 310` by `PID(Kp = 0.6, Ki = 0.2, Kd = 0.1)`, with `T_q` the
manipulated variable and `offset = 320`. Express this first-order plant as a `TransferFunction`
(or a custom `StateSpace`), run your simulator, and compare to the exported trajectory.

**Red — write the test:** `tests/control/notebook_golden_test.cc`:
- `test_matches_notebook_trajectory` *(golden)*: export the notebook's solution trajectory to a
  small committed file under `tests/control/data/`, run your simulator on the same grid and
  gains, and assert agreement to `1e-2`. Run → **red** until the data file exists and the plant
  mapping is in place.

**Green — make it pass:** add the exported reference data, then reconcile any constant-offset or
indexing differences — this is exactly where the notebook's `offset = 320` and the first-step
`Tₛ` guard earn their keep.

**Why it matters:** This is the honest system-level verdict — "does my from-scratch stack equal
the Python prototype I started from?"

**Gate:** `test_matches_notebook_trajectory` passes. **Faithfulness:** shift the setpoint by 5
units → the test goes red, proving the bar tracks the real trajectory rather than passing
vacuously.

**Commit:** `test(validation): reproduce notebook thermal-plant PID run`

### Step 8.2 — Second-order closed-loop matches analytic metrics

**Goal:** A genuinely second-order plant under PID hits the analytic overshoot/settling within
tolerance.

**What we're building (read this first).** Step 5.2 checked the loop at *steady state*; this step
checks its *transient shape* — the part PID tuning actually targets. You pick gains that place
the closed-loop poles at a known damping `ζ` and frequency `ωₙ`, simulate, and confirm the
measured overshoot and peak time match the analytic `Mp` and `tp`. You also check a *qualitative*
law that any correct PID must obey: turning up the derivative gain `Kd` should *reduce* overshoot.
Together these prove the loop is right not just where it ends up, but how it gets there.

**Control-theory specifics:** for a second-order closed loop with damping `ζ` and natural
frequency `ωₙ`, overshoot `Mp = exp(−πζ/√(1 − ζ²))` and peak time `tp = π/(ωₙ√(1 − ζ²))`
(Step 6.1); increasing `Kd` adds damping, which lowers overshoot.

**Red — write the test:** `tests/control/second_order_validation_test.cc`:
- `test_closed_loop_overshoot_tracks_theory` *(closed-form)*: **worked example** — choose gains
  giving `ζ = 0.5, ωₙ = 1` (i.e. a closed-loop characteristic polynomial `s² + s + 1`); simulate
  and assert the measured overshoot ≈ `16.3%` and peak time ≈ `3.63 s` within a few percent.
- `test_higher_kd_reduces_overshoot` *(behavioural)*: increasing `Kd` on a fixed plant
  monotonically lowers the measured overshoot.
- Build → **red**.

**Green — make it pass:** validation only — fix whatever the assertions expose.

**Why it matters:** Confirms the loop is right in its *transient* shape, not just at steady state.

**Gate:** both tests pass. **Faithfulness:** a deliberately mistuned (random) gain set must
*fail* the overshoot threshold, so the bar means something.

**Commit:** `test(validation): second-order overshoot/settling vs analytic`

---

## Phase 9 — Polish (optional but recommended)

### Step 9.1 — Transfer-function input from the command line / config

**Goal:** Let the user supply `numerator`, `denominator`, gains, setpoint, and horizon without
recompiling.

**What we're building (read this first).** Up to now the demo's plant is hard-coded in
`main.cc` — to try a different transfer function you edit C++ and rebuild. This step removes that
friction: parse the numerator, denominator, and gains from command-line arguments (or a config
file) and feed them into `runDemo()`. Now "type in a transfer function" is literal — the user
runs `MainExe` with arguments and gets a plot, no compiler involved.

**Control-theory specifics:** parse comma-separated coefficient lists into `Eigen::VectorXd`
(descending powers), reject inputs where the numerator degree exceeds the denominator degree
(improper).

**Red — write the test:** `tests/control/cli_test.cc`:
- `test_parses_tf_and_gains` *(contract)*: **worked example** — parsing `"1,3,2"` yields
  `den = [1, 3, 2]`; malformed input (e.g. `"1,,2"`) is rejected with a clear error.
- Build → **red**.

**Green — make it pass:** a small `argv`/config parser feeding `runDemo()`.

**Gate:** the parsing tests pass and `MainExe "5" "2,1" --kp 2 --ki 0.5` runs (the plant
`5/(2s + 1)`). **Faithfulness:** feed a numerator of higher degree than the denominator → it is
rejected as improper.

**Commit:** `feat(app): accept transfer function + gains as input`

### Step 9.2 — README + reproduce instructions

**Goal:** One page that builds, tests, and reproduces a Phase-8 plot on a clean checkout.

**What we're building (read this first).** The current [`README.md`](README.md) trails off into
placeholder text. This step makes the project reproducible by a stranger (or by you in six
months): a single page that explains the TF→PID→plot workflow, gives the exact build/test
commands for Linux *and* the existing Windows notes, and shows a sample output figure. The "test"
for this step is operational — the commands in the README must actually work on a clean checkout.

**Control-theory specifics:** none.

**Red — write the test:** the suite *is* the test — `ctest` green from a fresh `build/`, plus a
README "Reproduce" section whose commands you actually run and confirm.

**Green — make it pass:** fill in `README.md` (the workflow, the build/test commands, a sample
figure) and delete the placeholder text.

**Gate:** a clean-checkout `cmake … && ctest` is green and the README commands work as written.

**Commit:** `docs(readme): document build, test, and reproduce workflow`

### Step 9.3 — A small gallery of example plants

**Goal:** Ship a few ready-to-run plants so the demo shows the controller across regimes.

**What we're building (read this first).** A single example doesn't show what the controller can
do. This step ships a small gallery — an integrator `1/s`, a first-order lag, and under-/
over-damped second-order plants — each with sensible gains, so a new user can watch PID behave
across qualitatively different dynamics. A behavioural test guards them: every shipped example,
with its shipped gains, must actually settle to the setpoint within the horizon.

**Control-theory specifics:** each gallery entry is a `(TransferFunction, gains, horizon)` triple
chosen so the closed loop is stable and settles.

**Red — write the test:** `tests/control/gallery_test.cc`:
- `test_gallery_plants_are_stable_closed_loop` *(behavioural)*: **worked example** — each example,
  simulated with its shipped gains, ends within a small band of the setpoint.
- Build → **red**.

**Green — make it pass:** add the examples (a header table or small functions) and reference them
from the README.

**Gate:** the gallery test passes. **Faithfulness:** a deliberately under-damped gain set must
breach an overshoot bound, proving the test discriminates good tunings from bad.

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
