cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(PROJECT_NAME "assembly_cd")
project(${PROJECT_NAME})

#Add additional directory where to look for different python
# to enable compiling bindings for different python versions
if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/bin")
endif()

#Will try to find Python in the activated virtual environment directory
# first and then in other directories, returning the most recent version.
set(Python3_FIND_VIRTUALENV FIRST)
set(Python3_FIND_STRATEGY VERSION)

if(DEFINED ACD_BUILD_PYTHON_VERSION)
    if(ACD_BUILD_PYTHON_VERSION VERSION_LESS 3.9)
        message(FATAL_ERROR "Python version must be 3.9 or higher")
    elseif(ACD_BUILD_PYTHON_VERSION VERSION_LESS 3.10)
        find_package(Python3 3.9...<3.10 COMPONENTS Interpreter Development REQUIRED)
    elseif(ACD_BUILD_PYTHON_VERSION VERSION_LESS 3.11)
        find_package(Python3 3.10...<3.11 COMPONENTS Interpreter Development REQUIRED)
    elseif(ACD_BUILD_PYTHON_VERSION VERSION_LESS 3.12)
        find_package(Python3 3.11...<3.12 COMPONENTS Interpreter Development REQUIRED)
    elseif(ACD_BUILD_PYTHON_VERSION VERSION_LESS 3.13)
        find_package(Python3 3.12...<3.13 COMPONENTS Interpreter Development REQUIRED)
    else()
        message(FATAL_ERROR "Python version must be between 3.9 and 3.12")
    endif()
else()
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
endif()

add_subdirectory(${PYBIND_BUILD_DIR} ${CMAKE_BINARY_DIR}/pybind11)
pybind11_add_module(${PROJECT_NAME} MODULE src/PythonBindings.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${PROJECT_SOURCE_DIR}/../lib/src/ 
    ${PHYSX_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIR}
    ${OSQP_INCLUDE_DIR}
)

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
endif()

if (${Python3_FOUND})
    install(TARGETS ${PROJECT_NAME} DESTINATION ${Python3_STDLIB})
else()
    message(STATUS "WARNING: Python3 was not found and the bindings were not installed.")
endif()