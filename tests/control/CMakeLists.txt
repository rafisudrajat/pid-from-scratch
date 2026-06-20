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

add_executable(closed_loop_simulator_test closed_loop_simulator_test.cc)
target_link_libraries(closed_loop_simulator_test GTest::gtest_main CONTROL)
target_include_directories(closed_loop_simulator_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(step_response_metrics_test step_response_metrics_test.cc)
target_link_libraries(step_response_metrics_test GTest::gtest_main CONTROL)
target_include_directories(step_response_metrics_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(app_smoke_test app_smoke_test.cc)
target_link_libraries(app_smoke_test GTest::gtest_main CONTROL)
target_include_directories(app_smoke_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

add_executable(notebook_golden_test notebook_golden_test.cc)
target_link_libraries(notebook_golden_test GTest::gtest_main CONTROL)
target_include_directories(notebook_golden_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)
target_compile_definitions(notebook_golden_test PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

add_executable(second_order_validation_test second_order_validation_test.cc)
target_link_libraries(second_order_validation_test GTest::gtest_main CONTROL)
target_include_directories(second_order_validation_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/support/include
)

include(GoogleTest)
gtest_discover_tests(transfer_function_test)
gtest_discover_tests(state_space_test)
gtest_discover_tests(pid_controller_test)
gtest_discover_tests(closed_loop_simulator_test)
gtest_discover_tests(step_response_metrics_test)
gtest_discover_tests(app_smoke_test)
gtest_discover_tests(notebook_golden_test)
gtest_discover_tests(second_order_validation_test)
