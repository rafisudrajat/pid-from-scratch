cmake_minimum_required(VERSION 3.22)

project(
    PidFromScratch
    VERSION 1.0.0
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EXECUTABLE_NAME MainExe)

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(tests)

include(FetchContent)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/external_lib/eigen-lib-3.4.0)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/b514bdc898e2951020cbdca1304b75f5950d1f59.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
