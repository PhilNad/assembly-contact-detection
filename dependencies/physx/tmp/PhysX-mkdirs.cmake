# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX")
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX")
endif()
file(MAKE_DIRECTORY
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX-build"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/tmp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX-stamp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/physx/src/PhysX-stamp${cfgdir}") # cfgdir has leading slash
endif()
