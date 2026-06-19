add_executable(${EXECUTABLE_NAME} main.cc)

target_include_directories(${EXECUTABLE_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0
)

target_link_libraries(
    ${EXECUTABLE_NAME}
    PUBLIC CONTROL)

target_link_libraries(
    ${EXECUTABLE_NAME}
    PRIVATE matplotlib_lib)