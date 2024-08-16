add_executable(${EXECUTABLE_NAME} main.cc)

target_link_libraries(
    ${EXECUTABLE_NAME}
    PUBLIC MATH_OPERATION)
