cmake_minimum_required(VERSION 3.7)
cmake_policy(SET CMP0057 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

SET(PROJECT_NAME "assembly_cd")
project(${PROJECT_NAME})
pybind11_add_module(${PROJECT_NAME} MODULE src/PythonBindings.cpp)

set(PHYSX_ROOT_DIR ${PROJECT_SOURCE_DIR}/../physx)
set(PHYSX_BIN_PATH ${PHYSX_ROOT_DIR}/bin/linux.clang)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/../lib/src/ ${PHYSX_ROOT_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE assembly-cd-lib)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

#Debug or Release libraries are linked depending on the build type
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXExtensions_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysXExtensions_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysX_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysX_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXPvdSDK_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysXPvdSDK_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXCooking_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysXCooking_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXCommon_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysXCommon_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXFoundation_static_64.a optimized ${PHYSX_BIN_PATH}/release/libPhysXFoundation_static_64.a)

#This is needed only if you are running on the GPU
#target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPhysXGpu_64.so optimized ${PHYSX_BIN_PATH}/release/libPhysXGpu_64.so)

#This is needed only if you are using the PhysX Visual Debugger
target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPVDRuntime_64.so optimized ${PHYSX_BIN_PATH}/release/libPVDRuntime_64.so)


if (${Python3_FOUND})
    install(TARGETS ${PROJECT_NAME} DESTINATION ${Python3_STDLIB})
else()
    message(STATUS "WARNING: Python3 was not found and the bindings were not installed.")
endif()