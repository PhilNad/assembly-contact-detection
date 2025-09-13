# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp")
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp")
endif()
file(MAKE_DIRECTORY
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp-build"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/tmp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp-stamp"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src"
  "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/runner/work/assembly-contact-detection/assembly-contact-detection/dependencies/osqp/src/osqp-stamp${cfgdir}") # cfgdir has leading slash
endif()
