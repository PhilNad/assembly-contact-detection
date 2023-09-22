find_package (Eigen3 3.4 REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

SET(PROJECT_NAME "acd-tests")
project(${PROJECT_NAME})
add_executable(${PROJECT_NAME} src/test.cpp)

set(PHYSX_ROOT_DIR ${PROJECT_SOURCE_DIR}/../physx)
set(PHYSX_LIB_PATH ${PHYSX_ROOT_DIR}/bin/linux.clang/checked)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/../lib/src/ ${PHYSX_ROOT_DIR}/include)

set(PHYSX_LIBRARIES
    ${PHYSX_LIB_PATH}/libPhysXExtensions_static_64.a 
    ${PHYSX_LIB_PATH}/libPhysX_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXPvdSDK_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXVehicle_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXVehicle2_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXCharacterKinematic_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXCooking_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXCommon_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXFoundation_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXGpu_64.so
    #${PHYSX_LIB_PATH}/libPVDRuntime_64.so
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    assembly-cd-lib
    Eigen3::Eigen
    ${PHYSX_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
