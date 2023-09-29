find_package (Eigen3 3.4 REQUIRED NO_MODULE)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

SET(PROJECT_NAME "acd-tests")
project(${PROJECT_NAME})
add_executable(${PROJECT_NAME} src/test.cpp)

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
#target_link_libraries(${PROJECT_NAME} PRIVATE debug ${PHYSX_BIN_PATH}/checked/libPVDRuntime_64.so optimized ${PHYSX_BIN_PATH}/release/libPVDRuntime_64.so)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
