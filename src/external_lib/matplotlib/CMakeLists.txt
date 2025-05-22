# Set Python executable explicitly (adjust this path accordingly)
set(Python3_ROOT_DIR "C:/Users/user_name/miniconda3/envs/MLEnv")
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_FIND_ABI "cpython")  # Avoids _d suffix

set(LIBRARY_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/matplotlibcpp.h")
set(LIBRARY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/matplotlibcpp.cc")

# Find Python 3 (including NumPy)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

add_library(matplotlib_lib STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

# Include headers
target_include_directories(matplotlib_lib PUBLIC
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(matplotlib_lib PRIVATE ${Python3_LIBRARIES})

set_target_properties(matplotlib_lib PROPERTIES
    LINK_OPTIONS "/NODEFAULTLIB:python310_d"
)