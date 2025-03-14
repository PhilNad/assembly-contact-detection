include(ExternalProject)

set(EIGEN_BUILD_DIR   ${DEPS_CMAKE_DIRECTORY}/eigen/src/eigen)
set(EIGEN_INSTALL_DIR   ${DEPS_CMAKE_DIRECTORY}/eigen/src/eigen-build)
set(EIGEN_INCLUDE_DIR ${EIGEN_INSTALL_DIR}/include/eigen3)

#Eigen is a header only library.
ExternalProject_Add(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4/eigen-3.4.zip
    PREFIX ${DEPS_CMAKE_DIRECTORY}/eigen
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTALL_DIR} -DEIGEN_BUILD_TESTING=OFF
)