include(ExternalProject)

# Set the PhysX build type based on the CMake build type
# - debug: Used for PhysX error analysis, optimization turned off
# - checked: Used for development, optimization turned on
# - release: Used for release, optimization turned on
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PHYSX_BUILD_TYPE "checked")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(PHYSX_BUILD_TYPE "release")
endif()

set(PHYSX_BUILD_DIR   ${DEPS_CMAKE_DIRECTORY}/physx/src/PhysX/physx)
set(PHYSX_BINARY_DIR ${PHYSX_BUILD_DIR}/bin/linux.x86_64/${PHYSX_BUILD_TYPE})
set(PHYSX_INCLUDE_DIR ${PHYSX_BUILD_DIR}/include)

ExternalProject_Add(
    PhysX
    URL https://github.com/NVIDIA-Omniverse/PhysX/archive/refs/tags/106.5-physx-5.5.1-cy.zip
    PREFIX ${DEPS_CMAKE_DIRECTORY}/physx
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ${PHYSX_BUILD_DIR}/generate_projects.sh linux-gcc
    BUILD_COMMAND    
        COMMAND make --directory=${PHYSX_BUILD_DIR}/compiler/linux-gcc-${PHYSX_BUILD_TYPE} --file=${PHYSX_BUILD_DIR}/compiler/linux-gcc-${PHYSX_BUILD_TYPE}/Makefile -j${PROCESSOR_COUNT} LowLevel LowLevelAABB LowLevelDynamics PVDRuntime PhysX PhysXCharacterKinematic PhysXCommon PhysXCooking PhysXExtensions PhysXFoundation PhysXPvdSDK PhysXTask PhysXVehicle PhysXVehicle2 SceneQuery SimulationController
    INSTALL_COMMAND ""
    TEST_COMMAND ""
    LOG_DOWNLOAD ON
)

if(CMAKE_BUILD_TYPE AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(_DEBUG)
else()
    add_compile_definitions(NDEBUG)
endif()