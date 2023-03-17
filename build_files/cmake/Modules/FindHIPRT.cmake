# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2021 Blender Foundation.

# - Find HIP compiler
#
# This module defines
#  The path to HIP RT SDK
#

# If HIPRT_ROOT_DIR was defined in the environment, use it.
if(NOT HIPRT_ROOT_DIR AND NOT $ENV{HIPRT_ROOT_DIR} STREQUAL "")
  set(HIPRT_ROOT_DIR $ENV{HIPRT_ROOT_DIR})
endif()

set(_hiprt_SEARCH_DIRS
  ${HIPRT_ROOT_DIR}
)


FIND_PATH(HIPRT_INCLUDE_DIR
  NAMES
    hiprt/hiprt.h
    hiprt/hiprt_common.h
    hiprt/hiprt_device.h
    hiprt/hiprt_types.h
    hiprt/hiprt_vec.h
  HINTS
    ${_hiprt_SEARCH_DIRS}
  #PATH_SUFFIXES
   # hiprt
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(HIPRT DEFAULT_MSG
    HIPRT_INCLUDE_DIR)

IF(HIPRT_FOUND)
  SET(HIPRT_INCLUDE_DIR ${HIPRT_INCLUDE_DIR})
ELSE()
  SET(HIPRT_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
  HIPRT_INCLUDE_DIR
)
