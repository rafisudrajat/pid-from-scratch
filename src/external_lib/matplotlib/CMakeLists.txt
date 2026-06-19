# Always use the project-local .venv for Python (works on both Linux
# and Windows — create it with: python -m venv .venv && .venv/bin/pip
# install matplotlib numpy).
set(Python3_ROOT_DIR "${CMAKE_SOURCE_DIR}/.venv")
set(Python3_FIND_STRATEGY LOCATION)

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

# For the embedded Python interpreter to work at runtime, it needs:
#   1. PYTHONHOME — the base prefix where the standard library lives
#      (os.py, encodings/, etc.).  For a venv this is the base Python
#      installation the venv was created from, not the venv itself.
#   2. Site-packages path — so matplotlib and numpy are importable.
#
# We query both from the .venv's interpreter at configure time and bake
# them into the binary as compile definitions.  This avoids hardcoded
# paths and works on any machine that has a .venv in the project root.
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c
        "import sys; print(sys.base_prefix, end='')"
    OUTPUT_VARIABLE PYTHON3_BASE_PREFIX
)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c
        "import sysconfig; print(sysconfig.get_path('purelib'), end='')"
    OUTPUT_VARIABLE PYTHON3_SITE_PACKAGES
)
target_compile_definitions(matplotlib_lib PRIVATE
    PYTHON3_BASE_PREFIX="${PYTHON3_BASE_PREFIX}"
    PYTHON3_SITE_PACKAGES="${PYTHON3_SITE_PACKAGES}"
)
