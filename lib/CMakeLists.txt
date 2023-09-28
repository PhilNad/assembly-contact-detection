cmake_minimum_required(VERSION 3.7)

find_package (Eigen3 3.4 REQUIRED)

SET(PROJECT_NAME "assembly-cd-lib")
project(${PROJECT_NAME} VERSION 0.1.0 LANGUAGES C CXX)

file(GLOB SRC_LIB_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_LIB_FILES})

### Start of PhysX stuff
set(PHYSX_ROOT_DIR ${CMAKE_SOURCE_DIR}/physx)
set(PHYSX_LIB_PATH ${PHYSX_ROOT_DIR}/bin/linux.clang/checked)
include_directories(${PHYSX_ROOT_DIR}/include ${CMAKE_SOURCE_DIR}/lib/src)

set(PHYSX_LIBRARIES
    ${PHYSX_LIB_PATH}/libPhysXExtensions_static_64.a 
    ${PHYSX_LIB_PATH}/libPhysX_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXPvdSDK_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXVehicle_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXVehicle2_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXCharacterKinematic_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXCooking_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXCommon_static_64.a
    ${PHYSX_LIB_PATH}/libPhysXFoundation_static_64.a
    #${PHYSX_LIB_PATH}/libPhysXGpu_64.so
    #${PHYSX_LIB_PATH}/libPVDRuntime_64.so
)

#Use static libraries
add_compile_definitions(PX_PHYSX_STATIC_LIB)
#Do not use the GPU
#add_compile_definitions(DISABLE_CUDA_PHYSX)

# Specify whether the PhysX Snippets should be added to the build configuration - default True
set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
# Switch the PhysX build to output static libs instead of DLLs - default False
# If you choose to set this to True, then the first line in your main.cpp must be "#define PX_PHYSX_STATIC_LIB"
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
# Switch to precise math rather than fast math, this is beneficial especially for robotics projects, which do require higher precision - default False
set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "Float point precise math")
# Sets static or dynamic runtime usage - default True
set(NV_USE_STATIC_WINCRT OFF CACHE BOOL "Use the statically linked windows CRT")
# Specify whether debug CRT should be used - default True
set(NV_USE_DEBUG_WINCRT OFF CACHE BOOL "Use the debug version of the CRT")

### End of PhysX stuff

#Other libraries to use
set(MISC_LIBRARIES
    Eigen3::Eigen
    pthread
    dl
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

target_link_libraries(${PROJECT_NAME} ${MISC_LIBRARIES} ${PHYSX_LIBRARIES})