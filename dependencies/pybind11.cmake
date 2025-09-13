# Use the pybind11 submodule instead of ExternalProject
set(PYBIND_BUILD_DIR   ${CMAKE_SOURCE_DIR}/extern/pybind11)
set(PYBIND_INCLUDE_DIR ${PYBIND_BUILD_DIR}/include/pybind11)

# Create a dummy target for compatibility with the existing dependencies
add_custom_target(pybind11)