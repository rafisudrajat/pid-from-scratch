add_executable(transfer_function_test transfer_function_test.cc)
target_link_libraries(transfer_function_test GTest::gtest_main CONTROL)
target_include_directories(transfer_function_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

include(GoogleTest)
gtest_discover_tests(transfer_function_test)
