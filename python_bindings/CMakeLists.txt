cmake_minimum_required(VERSION 3.7)
cmake_policy(SET CMP0057 NEW)

find_package(Eigen3 3.4 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

SET(PROJECT_NAME "assembly-cd")
project(${PROJECT_NAME})
pybind11_add_module(${PROJECT_NAME} MODULE src/PythonBindings.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/../lib/src/)

target_link_libraries(${PROJECT_NAME} PRIVATE assembly-cd-lib)


if (${Python3_FOUND})
    install(TARGETS ${PROJECT_NAME} DESTINATION ${Python3_STDLIB})
else()
    message(STATUS "WARNING: Python3 was not found and the bindings were not installed.")
endif()