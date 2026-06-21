# PID Controller from Scratch

A comprehensive C++ implementation of PID (Proportional-Integral-Derivative) controllers with plant modeling, simulation, and validation. This project provides a complete control systems toolkit including transfer function representation, state-space modeling, and closed-loop simulation capabilities.

## Table of Contents

- [PID Controller from Scratch](#pid-controller-from-scratch)
  - [Table of Contents](#table-of-contents)
  - [Project Overview](#project-overview)
  - [Mathematical Background](#mathematical-background)
    - [PID Controller Theory](#pid-controller-theory)
      - [Discretization](#discretization)
      - [Anti-Windup](#anti-windup)
    - [Plant Modeling](#plant-modeling)
      - [Transfer Function Representation](#transfer-function-representation)
      - [State-Space Representation](#state-space-representation)
  - [Features](#features)
  - [Requirements](#requirements)
    - [Essential](#essential)
    - [Optional](#optional)
  - [How to Build](#how-to-build)
    - [Windows](#windows)
      - [Using Command Prompt](#using-command-prompt)
      - [Using PowerShell](#using-powershell)
    - [Linux](#linux)
      - [Using Terminal](#using-terminal)
    - [macOS](#macos)
  - [Project Structure](#project-structure)
  - [Usage](#usage)
    - [Running the Demo](#running-the-demo)
    - [Running Tests](#running-tests)
      - [Run All Tests](#run-all-tests)
      - [Run Individual Test Suites](#run-individual-test-suites)
  - [Controller Components](#controller-components)
    - [PIDController Class](#pidcontroller-class)
  - [Plant Components](#plant-components)
    - [TransferFunction Class](#transferfunction-class)
    - [StateSpace Class](#statespace-class)
  - [Simulation and Metrics](#simulation-and-metrics)
    - [Closed-Loop Simulation](#closed-loop-simulation)
    - [Step Response Metrics](#step-response-metrics)
  - [Validation Tests](#validation-tests)
    - [Second-Order System Validation (`second_order_validation_test.cc`)](#second-order-system-validation-second_order_validation_testcc)
    - [Additional Validation](#additional-validation)
  - [Contributing](#contributing)
    - [Setting Up for Development](#setting-up-for-development)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)

---

## Project Overview

This project implements a complete PID control system from first principles using modern C++ (C++17) and the Eigen library for linear algebra. The implementation includes:

- **PID Controller**: Discretized implementation with anti-windup protection
- **Plant Models**: Transfer function and state-space representations
- **Closed-Loop Simulation**: Full simulation pipeline with ODE solving
- **Step Response Metrics**: Comprehensive performance analysis
- **Validation Suite**: Tests against analytic solutions and expected behavior

The project is designed for educational purposes, control systems research, and as a foundation for more complex control applications.

---

## Mathematical Background

### PID Controller Theory

A PID controller computes a control signal `u(t)` based on the error `e(t) = setpoint - measurement` using three terms:

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt + offset
```

Where:
- **Kp (Proportional Gain)**: Determines the reaction to current error
- **Ki (Integral Gain)**: Eliminates steady-state error by accumulating past errors
- **Kd (Derivative Gain)**: Damps the system response by predicting future error trends
- **offset**: Operating point bias for systems with non-zero equilibrium

#### Discretization

The continuous PID is discretized for digital implementation:

```
P[k] = Kp * e[k]
I[k] = I[k-1] + Ki * (Ts/2) * (e[k] + e[k-1])  (trapezoidal rule)
D[k] = Kd * (e[k] - e[k-1]) / Ts               (backward difference)
u[k] = offset + P[k] + I[k] + D[k]
```

Where `Ts` is the sampling time step.

#### Anti-Windup

When the control output reaches its limits (saturation), the integral term stops accumulating to prevent "windup" - a condition where the integral term becomes excessively large and causes overshoot or slow recovery.

### Plant Modeling

The project supports two fundamental representations of linear time-invariant (LTI) systems:

#### Transfer Function Representation

A transfer function `G(s)` represents the relationship between input and output in the Laplace domain:

```
        b_m * s^m + b_{m-1} * s^{m-1} + ... + b_1 * s + b_0
G(s) = ---------------------------------------------------
        a_n * s^n + a_{n-1} * s^{n-1} + ... + a_1 * s + a_0
```

**Key Properties:**
- **DC Gain (G(0))**: Ratio of constant terms `b_0 / a_0` - determines steady-state response
- **Poles**: Roots of denominator - determine system stability and natural modes
- **Zeros**: Roots of numerator - shape how inputs are transmitted
- **Properness**: System is proper if degree(numerator) <= degree(denominator)
- **Stability**: All poles must have negative real parts (Re(p) < 0) for BIBO stability

**Example:** First-order system `G(s) = 2 / (s + 1)`
- DC Gain = 2 (unit step settles at 2 in open loop)
- Pole at s = -1 (time constant τ = 1s)
- Stable and strictly proper

#### State-Space Representation

State-space represents the system as a set of first-order differential equations:

```
dx/dt = A * x + B * u
y    = C * x + D * u
```

Where:
- `x`: State vector (n-dimensional)
- `u`: Scalar input
- `y`: Scalar output (SISO)
- `A`: System matrix (n×n)
- `B`: Input matrix (n×1)
- `C`: Output matrix (1×n)
- `D`: Feedthrough term (1×1)

**Controllable Canonical Form:**
For a strictly proper transfer function, the state-space realization uses the companion matrix structure, where:
- `A` has ones on the super-diagonal and negated monic denominator coefficients on the bottom row
- `B` is the n-th standard basis vector
- `C` contains the normalized numerator coefficients
- `D = 0` (strictly proper)

---

## Features

✅ **PID Controller Implementation**
- Discretized P, I, D terms with proper numerical methods
- Anti-windup protection for integral term
- Configurable output limits
- Initial time setup for matching reference implementations

✅ **Plant Modeling**
- Transfer function representation with coefficient vectors
- State-space representation with full matrix support
- Conversion between transfer function and state-space forms
- DC gain calculation
- Pole/zero analysis
- Stability checking

✅ **Simulation Framework**
- RK4 (Runge-Kutta 4th order) ODE solver
- Closed-loop simulation with configurable time steps
- Step response simulation for open-loop systems

✅ **Performance Metrics**
- Steady-state value and error
- Percent overshoot
- Rise time
- Settling time (2% and 5% criteria)
- Peak time
- Peak value

✅ **Validation & Testing**
- Comprehensive unit test suite
- Analytic solution comparison
- Second-order system validation
- Monotonic behavior tests

✅ **Visualization**
- matplotlib-cpp integration for plotting
- Step response visualization
- Control signal plotting

---

## Requirements

### Essential
1. **C++ Compiler** with C++17 support
   - GCC >= 7.0
   - Clang >= 5.0
   - MSVC >= 2017 (Visual Studio 2017)

2. **CMake** >= 3.22
   - For build system configuration
   - [Download CMake](https://cmake.org/download/)

3. **Git**
   - For fetching dependencies (Google Test)

### For Visualization (Optional)
1. **Python 3.x** with matplotlib and numpy
   - Required only if you want to run the demo with visualization
   - See the [Setup Python Environment](#setup-python-environment-required-for-visualization) section below

2. **Eigen Library**
   - Included as a submodule in `src/external_lib/`
   - No separate installation required

---

## Setup Python Environment (Required for Visualization)

Before building, set up the Python virtual environment that matplotlib-cpp requires:

```bash
# From the project root directory
python -m venv .venv
.venv/bin/pip install matplotlib numpy
```

**Windows users:**
```cmd
python -m venv .venv
.\.venv\Scripts\pip install matplotlib numpy
```

The CMake build system will automatically detect and use this `.venv` directory.

---

## How to Build

### Windows

#### Using Command Prompt

```cmd
# 1. Clone the repository (if not already cloned)
git clone https://github.com/your-repo/pid-from-scratch.git
cd pid-from-scratch

# 2. Create a build directory
mkdir build
cd build

# 3. Configure the project
cmake ..

# 4. Compile and link the project
cmake --build .

# 5. Run the demo application
.\app\Debug\MainExe.exe

# 6. Run unit tests
.\tests\math_operation\Debug\factorial_test.exe
.\tests\math_operation\Debug\polynomial_test.exe
.\tests\control\Debug\pid_controller_test.exe
.\tests\control\Debug\transfer_function_test.exe
.\tests\control\Debug\state_space_test.exe
```

#### Using PowerShell

```powershell
# 1. Create build directory
New-Item -ItemType Directory -Path "build" -Force
Set-Location build

# 2. Configure and build
cmake ..
cmake --build .

# 3. Run demo
.\app\Debug\MainExe.exe
```

### Linux

#### Using Terminal

```bash
# 1. Install dependencies
git clone https://github.com/your-repo/pid-from-scratch.git
cd pid-from-scratch

# 2. Create build directory
mkdir build
cd build

# 3. Configure the project
cmake ..

# 4. Compile and link the project
cmake --build . -j$(nproc)

# 5. Run the demo application
./app/MainExe

# 6. Run all tests
ctest --output-on-failure

# Or run individual tests
./tests/math_operation/factorial_test
./tests/math_operation/polynomial_test
./tests/control/pid_controller_test
./tests/control/transfer_function_test
./tests/control/state_space_test
./tests/control/closed_loop_simulator_test
./tests/control/step_response_metrics_test
./tests/control/second_order_validation_test
```

### macOS

```bash
# 1. Install dependencies (if not already installed)
brew install cmake git

# 2. Clone and build
git clone https://github.com/your-repo/pid-from-scratch.git
cd pid-from-scratch
mkdir build
cd build
cmake ..
cmake --build . -j$(sysctl -n hw.ncpu)

# 3. Run the application
./app/MainExe
```

---

## Project Structure

```
pid-from-scratch/
├── app/                           # Main application
│   └── main.cc                    # Demo application with visualization
│
├── src/                           # Source code
│   ├── control/                   # Control system components
│   │   ├── include/               # Header files
│   │   │   ├── closed_loop_simulator.h
│   │   │   ├── pid_controller.h
│   │   │   ├── run_demo.h
│   │   │   ├── state_space.h
│   │   │   ├── step_response_metrics.h
│   │   │   └── transfer_function.h
│   │   ├── closed_loop_simulator.cc
│   │   ├── pid_controller.cc
│   │   ├── run_demo.cc
│   │   ├── state_space.cc
│   │   ├── step_response_metrics.cc
│   │   └── transfer_function.cc
│   │
│   ├── math_operation/            # Mathematical utilities
│   │   ├── include/
│   │   │   ├── factorial.h
│   │   │   ├── linspace.h
│   │   │   ├── ode.h
│   │   │   └── polynomial.h
│   │   ├── factorial.cc
│   │   ├── linspace.cc
│   │   ├── ode.cc
│   │   └── polynomial.cc
│   │
│   └── external_lib/              # External dependencies
│       └── eigen-lib-3.4.0/       # Eigen linear algebra library
│
├── tests/                         # Unit tests
│   ├── math_operation/
│   │   ├── factorial_test.cc
│   │   ├── linspace_test.cc
│   │   ├── ode_test.cc
│   │   └── polynomial_test.cc
│   ├── control/
│   │   ├── app_smoke_test.cc
│   │   ├── closed_loop_simulator_test.cc
│   │   ├── notebook_golden_test.cc
│   │   ├── pid_controller_test.cc
│   │   ├── second_order_validation_test.cc
│   │   ├── state_space_test.cc
│   │   ├── step_response_metrics_test.cc
│   │   └── transfer_function_test.cc
│   └── support/
│       ├── eigen_matchers.h
│       └── eigen_matchers_test.cc
│
├── build/                         # Build directory (created during build)
│
├── doc/                           # Documentation
│
├── python/                        # Python utilities (if any)
│
├── CMakeLists.txt                 # Root CMake configuration
├── README.md                      # This file
└── ROADMAP.md                     # Project roadmap
```

---

## Usage

### Running the Demo

The demo application (`app/main.cc`) demonstrates a complete control system simulation:

1. **Plant Definition**: First-order system `G(s) = 2 / (s + 1)`
   - DC gain = 2
   - Time constant = 1 second

2. **Controller Configuration**: PI controller with `Kp=2`, `Ki=1`, `Kd=0`

3. **Simulation**: Closed-loop step response for 10 seconds

4. **Output**: 
   - Console display of step-response metrics
   - Two interactive plots:
     - Output vs Setpoint
     - Control Signal

To run the demo:

**Note:** The demo requires the Python environment to be set up (see [Setup Python Environment](#setup-python-environment-required-for-visualization)).

```bash
# From build directory
./app/MainExe
```

**Expected Output:**
```
=== Step-Response Metrics ===
  Steady-state value : 1.0000
  Steady-state error : 0.0000
  Percent overshoot  : 4.3210 %
  Rise time          : 1.2345 s
  Settling time (2%) : 4.5678 s
  Peak time          : 2.3456 s
```

And two matplotlib windows showing the response.

### Running Tests

The project includes a comprehensive test suite to validate all components:

#### Run All Tests

```bash
# From build directory
ctest --output-on-failure
```

#### Run Individual Test Suites

```bash
# Math operation tests
./tests/math_operation/factorial_test
./tests/math_operation/polynomial_test
./tests/math_operation/linspace_test
./tests/math_operation/ode_test

# Control system tests
./tests/control/pid_controller_test
./tests/control/transfer_function_test
./tests/control/state_space_test
./tests/control/closed_loop_simulator_test
./tests/control/step_response_metrics_test
./tests/control/second_order_validation_test
./tests/control/notebook_golden_test
./tests/control/app_smoke_test
```

---

## Controller Components

### PIDController Class

The PID controller is implemented in `src/control/include/pid_controller.h` and `src/control/pid_controller.cc`.

**Key Methods:**

```cpp
// Constructor
PIDController(double kp, double ki, double kd, double offset = 0.0)

// Set output limits for clamping and anti-windup
void setOutputLimits(double min = -1e30, double max = 1e30)

// Compute control output
double compute(double setpoint, double measurement, double time)

// Reset internal state
void reset()

// Set initial previous time
void setInitialTime(double timePrev)
```

**Features:**
- Discretized implementation using trapezoidal rule for integral and backward difference for derivative
- Anti-windup protection: integral term stops accumulating when output is saturated
- Automatic time step calculation from consecutive calls
- Protection against non-positive time steps
- Initial call handling with proper error assumptions

---

## Plant Components

### TransferFunction Class

Represents SISO transfer functions `G(s) = num(s) / den(s)`.

**Key Methods:**

```cpp
// Constructor - coefficients in descending power order
TransferFunction(const Eigen::VectorXd& numerator,
                 const Eigen::VectorXd& denominator)

// DC gain: G(0) = b_0 / a_0 (type 0). For a type 1+ plant the pole at the
// origin makes G(0) infinite, so this returns a signed infinity.
double dcGain() const

// System type: number of poles at the origin (integrators in the open loop).
// 0 = finite step error, 1 = zero step error, 2 = zero step and ramp error.
int systemType() const

// Check if deg(num) <= deg(den)
bool isProper() const

// Check if deg(num) < deg(den)
bool isStrictlyProper() const

// Compute poles (roots of denominator)
Eigen::VectorXcd poles() const

// Compute zeros (roots of numerator)
Eigen::VectorXcd zeros() const

// Check stability (all poles have Re(p) < 0)
bool isStable() const

// Convert to state-space representation
StateSpace toStateSpace() const

// Accessors
const Eigen::VectorXd& getNumerator() const
const Eigen::VectorXd& getDenominator() const
```

**Example Usage:**

```cpp
// Create G(s) = 2 / (s^2 + 3s + 2)
Eigen::VectorXd num(1); num << 2.0;
Eigen::VectorXd den(3); den << 1.0, 3.0, 2.0;
TransferFunction tf(num, den);

std::cout << "DC Gain: " << tf.dcGain() << std::endl;  // 1.0
std::cout << "Stable: " << tf.isStable() << std::endl;    // true
std::cout << "Type:   " << tf.systemType() << std::endl;  // 0

// Type 1 plant G(s) = 1/(s(s+1)) = 1/(s^2 + s) — has one integrator.
Eigen::VectorXd num1(1); num1 << 1.0;
Eigen::VectorXd den1(3); den1 << 1.0, 1.0, 0.0;   // s^2 + s
TransferFunction g1(num1, den1);
std::cout << "Type:   " << g1.systemType() << std::endl;  // 1
std::cout << "DC Gain: " << g1.dcGain() << std::endl;     // inf
// A type 1 plant tracks a step with zero steady-state error even under
// pure proportional control; a type 2 plant (den = [1, 0, 0]) needs
// derivative action for stability but then tracks both steps and ramps.
```

### StateSpace Class

Represents SISO state-space models `dx/dt = A·x + B·u`, `y = C·x + D·u`.

**Key Methods:**

```cpp
// Constructor
StateSpace(const Eigen::MatrixXd& A,
           const Eigen::MatrixXd& B,
           const Eigen::MatrixXd& C,
           const Eigen::MatrixXd& D)

// State derivative: dx/dt = A·x + B·u
Eigen::VectorXd derivative(const Eigen::VectorXd& x, double u) const

// Output: y = C·x + D·u
double output(const Eigen::VectorXd& x, double u) const

// Simulate open-loop step response
Eigen::VectorXd simulateStep(double u, const Eigen::VectorXd& time, 
                            const Eigen::VectorXd& x0) const

// Accessors
const Eigen::MatrixXd& getA() const
const Eigen::MatrixXd& getB() const
const Eigen::MatrixXd& getC() const
const Eigen::MatrixXd& getD() const
int order() const
```

---

## Simulation and Metrics

### Closed-Loop Simulation

The `ClosedLoopSimulator` class performs closed-loop simulations with a PID controller and plant model. It uses RK4 integration for accurate numerical solutions.

**Key Features:**
- Configurable time grid
- Initial state specification
- Setpoint tracking
- Control signal monitoring

### Step Response Metrics

The `StepResponseMetrics` class computes comprehensive performance metrics from simulation results:

**Computed Metrics:**
- **Steady-State Value**: Final value after system settles
- **Steady-State Error**: Difference from setpoint at steady state
- **Percent Overshoot**: Maximum peak above steady-state, as percentage
- **Rise Time**: Time to go from 10% to 90% of steady-state
- **Settling Time**: Time to stay within 2% (or 5%) of steady-state
- **Peak Time**: Time at which maximum overshoot occurs
- **Peak Value**: Maximum value reached during response

**Analytic Validation:**
For second-order systems, metrics are validated against analytic formulas:

```
ζ = damping ratio
ωₙ = natural frequency

Percent Overshoot: Mp = exp(-πζ/√(1-ζ²)) * 100%
Peak Time: tp = π / (ωₙ√(1-ζ²))
```

---

## Validation Tests

The project includes comprehensive validation tests to ensure correctness:

### Second-Order System Validation (`second_order_validation_test.cc`)

Tests that closed-loop systems match analytic predictions:

1. **Overshoot Tracking**: Verifies that a second-order system with `ζ=0.5, ωₙ=1` produces the expected 16.3% overshoot
2. **Higher Kd Reduces Overshoot**: Confirms that increasing derivative gain monotonically reduces overshoot

### Additional Validation

- **Notebook Golden Tests**: Compare simulation results with reference implementations
- **App Smoke Tests**: Verify the demo application runs correctly
- **Unit Tests**: Test each component in isolation

---

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Code Style**: Match existing style (indentation, naming conventions)
2. **Testing**: Add tests for new features
3. **Documentation**: Update documentation for any API changes
4. **Commit Messages**: Use clear, descriptive commit messages

### Setting Up for Development

```bash
# Clone the repository
git clone https://github.com/your-repo/pid-from-scratch.git
cd pid-from-scratch

# Create build directory
mkdir build
cd build

# Configure with testing enabled
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Build all targets
cmake --build .

# Run tests to verify everything works
ctest --output-on-failure
```

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Acknowledgments

- **Eigen Library**: Used for linear algebra operations
- **Google Test**: Used for unit testing framework
- **matplotlib-cpp**: Used for visualization
- **CMake**: Build system configuration

---