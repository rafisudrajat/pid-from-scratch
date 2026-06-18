enable_testing()

add_executable(
    factorial_test
    factorial_test.cc
)

target_link_libraries(
    factorial_test
    GTest::gtest_main
    MATH_OPERATION
)

add_executable(
    ode_test
    ode_test.cc
)

target_link_libraries(
    ode_test
    GTest::gtest_main
    MATH_OPERATION
)

target_include_directories(ode_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(linspace_test linspace_test.cc)
target_link_libraries(linspace_test GTest::gtest_main MATH_OPERATION)

add_executable(polynomial_test polynomial_test.cc)
target_link_libraries(polynomial_test GTest::gtest_main MATH_OPERATION)

include(GoogleTest)
gtest_discover_tests(factorial_test)
gtest_discover_tests(ode_test)
gtest_discover_tests(linspace_test)
gtest_discover_tests(polynomial_test)
