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

include(GoogleTest)
gtest_discover_tests(factorial_test)
