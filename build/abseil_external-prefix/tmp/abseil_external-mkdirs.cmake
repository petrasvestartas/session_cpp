# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external")
  file(MAKE_DIRECTORY "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external")
endif()
file(MAKE_DIRECTORY
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external-build"
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix"
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/tmp"
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external-stamp"
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src"
  "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/petras/code/code_rust/session/session_cpp/build/abseil_external-prefix/src/abseil_external-stamp${cfgdir}") # cfgdir has leading slash
endif()
