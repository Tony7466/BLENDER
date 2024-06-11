# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Note the utility apps may use png/tiff/gif system libraries, but the
# library itself does not depend on them, so should give no problems.

set(HIPRT_CMAKE_FLAGS ${DEFAULT_CMAKE_FLAGS})

get_filename_component(_hip_path ${HIP_HIPCC_EXECUTABLE} DIRECTORY)
get_filename_component(_hip_path ${_hip_path} DIRECTORY)

set(HIPRT_EXTRA_ARGS
  -DCMAKE_BUILD_TYPE=Release
  -DHIP_PATH=${_hip_path}
  -DBITCODE=ON
)

ExternalProject_Add(external_hiprt
  URL file://${PACKAGE_DIR}/${HIPRT_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${HIPRT_HASH_TYPE}=${HIPRT_HASH}
  PREFIX ${BUILD_DIR}/hiprt
  INSTALL_DIR ${LIBDIR}/hiprt

  CMAKE_ARGS ${DEFAULT_CMAKE_ARGS} ${HIPRT_EXTRA_ARGS} 
)

# TODO: check installed files are as expected.

if(WIN32)
  if(BUILD_MODE STREQUAL Release)
    ExternalProject_Add_Step(external_hiprt after_install
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${LIBDIR}/hiprt
        ${HARVEST_TARGET}/hiprt
      DEPENDEES install
    )
  endif()
endif()
