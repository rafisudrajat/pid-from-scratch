add_executable(eigen_matchers_test eigen_matchers_test.cc)

target_link_libraries(eigen_matchers_test GTest::gtest_main)

target_include_directories(eigen_matchers_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0)

include(GoogleTest)
gtest_discover_tests(eigen_matchers_test)