include(GNUInstallDirs)
cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

SET(PROJECT_NAME "acd-tests")
project(${PROJECT_NAME})
add_executable(${PROJECT_NAME} src/test.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${PHYSX_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/lib/src ${EIGEN_INCLUDE_DIR} ${OSQP_INCLUDE_DIR})

#Link to the library
target_link_libraries(${PROJECT_NAME} PRIVATE assembly-cd-lib)

#Link to PhysX libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXExtensions_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysX_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXPvdSDK_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXCooking_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXCommon_static_64.a)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXFoundation_static_64.a)

if(ACD_USE_GPU)
    #This is needed only if you are running on the GPU
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPhysXGpu_64.so)
endif()

if(ACD_USE_PVD)
    #This is needed only if you are using the PhysX Visual Debugger
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PHYSX_BINARY_DIR}/libPVDRuntime_64.so)
    install(FILES ${PHYSX_BINARY_DIR}/libPVDRuntime_64.so DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()