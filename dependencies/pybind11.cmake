include(ExternalProject)

set(PYBIND_BUILD_DIR   ${DEPS_CMAKE_DIRECTORY}/pybind11/src/pybind11)
set(PYBIND_INCLUDE_DIR ${PYBIND_BUILD_DIR}/include/pybind11)

#Pybind options
set(PYBIND_BUILD_TESTS OFF CACHE BOOL "Build pybind11 test suite")

ExternalProject_Add(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG stable
    PREFIX ${DEPS_CMAKE_DIRECTORY}/pybind11
    CMAKE_ARGS -DPYBIND11_TEST=${PYBIND_BUILD_TESTS} -DPYBIND11_INSTALL=OFF
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    BUILD_IN_SOURCE ON
)