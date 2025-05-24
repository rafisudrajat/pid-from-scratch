# Gather all source and header files in math_operation
set(LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/factorial.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/ode.cc
)

set(LIBRARY_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/factorial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ode.h
)

set(LIBRARY_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Define the MATH_OPERATION library
add_library(MATH_OPERATION STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

# Add include directories for this library
target_include_directories(MATH_OPERATION PUBLIC ${LIBRARY_INCLUDES})
