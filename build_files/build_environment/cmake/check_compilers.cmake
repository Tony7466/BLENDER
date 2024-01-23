# SPDX-FileCopyrightText: 2019-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Compiler version used for precompiled library builds used for official releases.

SET(RELEASE_GCC_VERSION 11.2.*)
SET(RELEASE_CUDA_VERSION 12.3.*)
SET(RELEASE_HIP_VERSION 5.7.*)

# Check against installed versions.

message(STATUS "Found C Compiler: ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}")
if(UNIX AND NOT APPLE)
  if(NOT CMAKE_COMPILER_IS_GNUCC OR NOT (CMAKE_C_COMPILER_VERSION MATCHES ${RELEASE_GCC_VERSION}))
    message(STATUS "  NOTE: Official releases uses GCC ${RELEASE_GCC_VERSION}")
  endif()
endif()

message(STATUS "Found C++ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
if(UNIX AND NOT APPLE)
  if(NOT CMAKE_COMPILER_IS_GNUCC OR NOT (CMAKE_CXX_COMPILER_VERSION MATCHES ${RELEASE_GCC_VERSION}))
    message(STATUS "  NOTE: Official releases uses GCC ${RELEASE_GCC_VERSION}")
  endif()
endif()

if(NOT APPLE)
  include(CheckLanguage)
  check_language(CUDA)
  if (NOT CMAKE_CUDA_COMPILER)
    message(STATUS "Missing CUDA compiler")
  else()
    enable_language(CUDA)
    message(STATUS "Found CUDA Compiler: ${CMAKE_CUDA_COMPILER_ID} ${CMAKE_CUDA_COMPILER_VERSION}")
    if(NOT CMAKE_CUDA_COMPILER_VERSION MATCHES ${RELEASE_CUDA_VERSION})
      message(STATUS "  NOTE: Official releases uses CUDA ${RELEASE_CUDA_VERSION}")
    endif()
  endif()

  check_language(HIP)
  if (NOT CMAKE_HIP_COMPILER)
    message(STATUS "Missing HIP compiler")
  else()
    enable_language(HIP)
    message(STATUS "Found HIP Compiler: ${CMAKE_HIP_COMPILER_ID} ${CMAKE_HIP_COMPILER_VERSION}")
    if(NOT CMAKE_HIP_COMPILER_VERSION MATCHES ${RELEASE_HIP_VERSION})
      message(STATUS "  NOTE: Official releases uses HIP ${RELEASE_HIP_VERSION}")
    endif()
  endif()
endif()
