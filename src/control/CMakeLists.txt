set(LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/transfer_function.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/state_space.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/pid_controller.cc
)

set(LIBRARY_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/transfer_function.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/state_space.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/pid_controller.h
)

set(LIBRARY_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(CONTROL STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

target_include_directories(CONTROL PUBLIC
    ${LIBRARY_INCLUDES}
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0
)

target_link_libraries(CONTROL PUBLIC MATH_OPERATION)
