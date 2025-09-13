cmake_minimum_required(VERSION 3.18)

# set the project name
project(assembly-contact-detection)

# Specify the Python version that the bindings will be built for.
#   Versions currently supported: 3.9, 3.10, 3.11, 3.12
set(ACD_BUILD_PYTHON_VERSION 3.12)

# Specify wheather the build should include tests or not
set(ACD_DO_TESTS ON)

# Specify wheather to build unit tests with gtest
option(ACD_BUILD_GTEST_TESTS "Build unit tests with Google Test framework" ON)

# Specify wheather the build should rely on GPU or not
set(ACD_USE_GPU OFF)

# Specify wheather the build should stream to the PhysX Visual Debugger or not
set(ACD_USE_PVD OFF)

# Directory containing CMAKE files that add external dependencies
set(DEPS_CMAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/dependencies)

# Include requisite dependencies
include(${DEPS_CMAKE_DIRECTORY}/eigen.cmake)
include(${DEPS_CMAKE_DIRECTORY}/osqp.cmake)
include(${DEPS_CMAKE_DIRECTORY}/physx.cmake)
include(${DEPS_CMAKE_DIRECTORY}/pybind11.cmake)

# Include gtest if unit tests are enabled
if(ACD_BUILD_GTEST_TESTS)
    include(${DEPS_CMAKE_DIRECTORY}/gtest.cmake)
endif()

add_dependencies(pybind11 pybind11)
add_dependencies(eigen eigen)
add_dependencies(osqp osqp)
add_dependencies(PhysX PhysX)

# Add gtest dependency if enabled
if(ACD_BUILD_GTEST_TESTS)
    add_dependencies(googletest googletest)
endif()

add_subdirectory(lib)
add_subdirectory(python_bindings)
if(ACD_DO_TESTS)
    add_subdirectory(test)
endif()