include(GNUInstallDirs)
cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(PROJECT_NAME "assembly-cd-lib")
project(${PROJECT_NAME} VERSION 0.2.0 LANGUAGES C CXX)

#Find and include all source files
file(GLOB SRC_LIB_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_LIB_FILES})

#Include directories
include_directories(
    ${PHYSX_INCLUDE_DIR} 
    ${CMAKE_SOURCE_DIR}/lib/src
    ${OSQP_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
)

#Use static libraries
add_compile_definitions(PX_PHYSX_STATIC_LIB)

if(NOT ACD_USE_GPU)
    add_compile_definitions(DISABLE_CUDA_PHYSX)
endif()

# Specify whether the PhysX Snippets should be added to the build configuration - default True
set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
# Switch the PhysX build to output static libs instead of DLLs - default False
# If you choose to set this to True, then the first line in your main.cpp must be "#define PX_PHYSX_STATIC_LIB"
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
# Switch to precise math rather than fast math, this is beneficial especially for robotics projects, which do require higher precision - default False
set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "Float point precise math")

#Other libraries to use
set(MISC_LIBRARIES
    pthread
    dl
)

#Use static OSQP library to make it easier to distribute
#target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqpstatic)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OSQP_BINARY_DIR}/libosqpstatic.a)

target_link_libraries(${PROJECT_NAME} PRIVATE ${MISC_LIBRARIES})

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

install(TARGETS ${PROJECT_NAME})