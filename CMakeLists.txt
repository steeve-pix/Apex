cmake_minimum_required(VERSION 3.24)

project(Apex VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
        GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(glfw)

add_executable(apex src/main.cpp)

target_link_libraries(apex PRIVATE glfw)

target_compile_features(apex PRIVATE cxx_std_20)

set_target_properties(apex PROPERTIES CXX_EXTENSIONS OFF)

if (APPLE)
    include(cmake/MacOSBundle.cmake)
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(apex PRIVATE -Wall -Wextra -Wpedantic)
endif ()
