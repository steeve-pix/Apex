cmake_minimum_required(VERSION 3.24)

project(Apex VERSION 0.1.0 LANGUAGES C CXX)

if (APPLE)
    enable_language(OBJC OBJCXX)
endif ()

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

FetchContent_Declare(
        glad
        GIT_REPOSITORY https://github.com/Dav1dde/glad.git
        GIT_TAG v2.0.8
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR cmake
)

FetchContent_MakeAvailable(glad)

glad_add_library(apex_glad STATIC
        REPRODUCIBLE
        API gl:core=3.3
        EXTENSIONS NONE
)
add_library(apex_simulation STATIC
        src/simulation/body.cpp
        src/simulation/vehicle.cpp
        src/simulation/track.cpp
        src/simulation/world.cpp
        src/simulation/driver.cpp
        src/simulation/sensors.cpp
)

target_include_directories(apex_simulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_compile_features(apex_simulation PUBLIC cxx_std_20)

add_library(apex_graphics STATIC
        src/platform/glfw_session.cpp
        src/graphics/shader.cpp
        src/graphics/shader_program.cpp
        src/graphics/triangle.cpp
        src/graphics/track_mesh.cpp
        src/graphics/sensor_lines.cpp
        src/graphics/simulation_window.cpp
)

target_link_libraries(apex_graphics PUBLIC
        glfw
        apex_glad
        apex_simulation
)

if (APPLE)
    enable_language(OBJCXX)

    target_sources(apex_graphics PRIVATE
            src/platform/macos_application.mm
    )

    target_link_libraries(apex_graphics PRIVATE "-framework Cocoa")
endif ()

add_executable(apex src/main.cpp)
target_link_libraries(apex PRIVATE apex_graphics)

option(APEX_BUILD_PYTHON_BINDINGS "Build Python simulation bindings" OFF)

if (APEX_BUILD_PYTHON_BINDINGS)
    find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

    execute_process(
            COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
            OUTPUT_VARIABLE apex_pybind11_cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ERROR_IS_FATAL ANY
    )
    find_package(pybind11 CONFIG REQUIRED
            PATHS "${apex_pybind11_cmake_dir}"
            NO_DEFAULT_PATH
    )

    set_target_properties(
            apex_simulation apex_graphics apex_glad glfw
            PROPERTIES POSITION_INDEPENDENT_CODE ON
    )

    pybind11_add_module(apex_core src/bindings/python_module.cpp)
    target_link_libraries(apex_core PRIVATE apex_graphics)
endif ()

add_executable(apex_headless src/headless_main.cpp)
target_link_libraries(apex_headless PRIVATE apex_simulation)

set_target_properties(apex PROPERTIES CXX_EXTENSIONS OFF)

if (APPLE)
    include(cmake/MacOSBundle.cmake)
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(apex PRIVATE -Wall -Wextra -Wpedantic)
endif ()


include(CTest)

if (BUILD_TESTING)
    add_executable(physics_tests
            tests/physics_tests.cpp
    )

    target_link_libraries(physics_tests PRIVATE apex_simulation)

    add_test(NAME physics COMMAND physics_tests)
endif ()
