# pid-from-scratch

This project simulate PID controller from scratch using C++

## Requirement:
1. C++ compiler
2. CMake

## How to build
### Windows
1. Create a build directory
   ```bat
    mkdir build
   ```
2. Configure the project
   ```bat
   cd build
   cmake ../
   ```
3. Compile and link the project
   ```bat
   cmake --build .
   ```
4. Run unit test
   ```bat
   <!-- From build directory -->
   .\tests\math_operation\Debug\factorial_test.exe
   ```
   The test should pass all the test cases