# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11")
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11")
endif()
file(MAKE_DIRECTORY
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11-build"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/tmp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11-stamp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/pybind11/src/pybind11-stamp${cfgdir}") # cfgdir has leading slash
endif()
