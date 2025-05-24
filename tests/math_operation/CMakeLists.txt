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

include(GoogleTest)
gtest_discover_tests(factorial_test)
gtest_discover_tests(ode_test)
