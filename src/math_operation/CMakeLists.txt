# Gather all source and header files in math_operation
set(LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/factorial.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/ode.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/linspace.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/polynomial.cc
)

set(LIBRARY_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/factorial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/linspace.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/polynomial.h
)

set(LIBRARY_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Define the MATH_OPERATION library
add_library(MATH_OPERATION STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

# Add include directories for this library
target_include_directories(MATH_OPERATION 
    PUBLIC 
    ${LIBRARY_INCLUDES}
    ${CMAKE_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0
)
