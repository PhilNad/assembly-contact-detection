include(ExternalProject)

# Set up Google Test as an external project
set(GTEST_BUILD_DIR   ${DEPS_CMAKE_DIRECTORY}/googletest/src/googletest)
set(GTEST_INCLUDE_DIR ${GTEST_BUILD_DIR}/googletest/include)
set(GTEST_BINARY_DIR  ${DEPS_CMAKE_DIRECTORY}/googletest/src/googletest-build)

# GTest options
set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib even when Google Test is built as static lib.")
set(BUILD_GMOCK OFF CACHE BOOL "Builds the googlemock subproject")
set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest")

ExternalProject_Add(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    PREFIX ${DEPS_CMAKE_DIRECTORY}/googletest
    CMAKE_ARGS 
        -Dgtest_force_shared_crt=${gtest_force_shared_crt}
        -DBUILD_GMOCK=${BUILD_GMOCK}
        -DINSTALL_GTEST=${INSTALL_GTEST}
        -DCMAKE_BUILD_TYPE=Release
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
)

# Set up variables for linking
set(GTEST_LIBRARIES ${GTEST_BINARY_DIR}/lib/libgtest.a ${GTEST_BINARY_DIR}/lib/libgtest_main.a)