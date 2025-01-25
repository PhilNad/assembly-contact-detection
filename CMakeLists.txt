cmake_minimum_required(VERSION 3.18)

# set the project name
project(assembly-contact-detection)

# Specify the Python version that the bindings will be built for.
#   Versions currently supported: 3.9, 3.10, 3.11, 3.12
set(ACD_BUILD_PYTHON_VERSION 3.9)

add_subdirectory(lib)
add_subdirectory(test)
add_subdirectory(python_bindings)