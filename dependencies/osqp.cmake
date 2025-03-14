include(ExternalProject)

set(OSQP_BUILD_DIR   ${DEPS_CMAKE_DIRECTORY}/osqp/src/osqp)
set(OSQP_INCLUDE_DIR ${OSQP_BUILD_DIR}/include/public)
set(OSQP_BINARY_DIR ${DEPS_CMAKE_DIRECTORY}/osqp/src/osqp-build/out)

#OSQP options
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "Build shared library")
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "Build static library")
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "Build demo executable")
set(OSQP_CODEGEN OFF CACHE BOOL "Build codegen")

ExternalProject_Add(
    osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v1.0.0.beta1
    PREFIX ${DEPS_CMAKE_DIRECTORY}/osqp
    CMAKE_GENERATOR "Unix Makefiles"
    CMAKE_ARGS -DOSQP_BUILD_SHARED_LIB=${OSQP_BUILD_SHARED_LIB} -DOSQP_BUILD_STATIC_LIB=${OSQP_BUILD_STATIC_LIB} -DOSQP_BUILD_DEMO_EXE=${OSQP_BUILD_DEMO_EXE} -DOSQP_CODEGEN=${OSQP_CODEGEN}
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    BUILD_IN_SOURCE ON
)