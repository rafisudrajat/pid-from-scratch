add_executable(transfer_function_test transfer_function_test.cc)
target_link_libraries(transfer_function_test GTest::gtest_main CONTROL)
target_include_directories(transfer_function_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(state_space_test state_space_test.cc)
target_link_libraries(state_space_test GTest::gtest_main CONTROL)
target_include_directories(state_space_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(pid_controller_test pid_controller_test.cc)
target_link_libraries(pid_controller_test GTest::gtest_main CONTROL)
target_include_directories(pid_controller_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

include(GoogleTest)
gtest_discover_tests(transfer_function_test)
gtest_discover_tests(state_space_test)
gtest_discover_tests(pid_controller_test)
