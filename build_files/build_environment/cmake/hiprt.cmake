# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Note the utility apps may use png/tiff/gif system libraries, but the
# library itself does not depend on them, so should give no problems.

if(NOT (BUILD_MODE STREQUAL Release))
  return()
endif()

set(HIPRT_CMAKE_FLAGS ${DEFAULT_CMAKE_FLAGS})

get_filename_component(_hip_path ${HIP_HIPCC_EXECUTABLE} DIRECTORY)
get_filename_component(_hip_path ${_hip_path} DIRECTORY)

set(HIPRT_EXTRA_ARGS
  -DCMAKE_BUILD_TYPE=Release
  -DHIP_PATH=${_hip_path}
  -DBITCODE=ON
  -DNO_UNITTEST=ON
  -DPRECOMPILE=ON
  -DHIPRT_PREFER_HIP_5=ON
)

set(HIPRT_SOURCE_DIR ${BUILD_DIR}/hiprt/src/external_hiprt)
set(HIPRT_BUILD_DIR ${BUILD_DIR}/hiprt/src/external_hiprt-build)

# Work around relative paths in bake kernel script and missing
# executable permission on encryption binary.
if(WIN32)
  set(HIPRT_WORKAROUND
    cd ${HIPRT_SOURCE_DIR} &&
  )
else()
  set(HIPRT_WORKAROUND
    cd ${HIPRT_SOURCE_DIR} &&
    chmod +x ./contrib/easy-encryption/bin/linux/ee64 &&
  )
endif()

ExternalProject_Add(external_hiprt
  URL file://${PACKAGE_DIR}/${HIPRT_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${HIPRT_HASH_TYPE}=${HIPRT_HASH}
  PREFIX ${BUILD_DIR}/hiprt
  INSTALL_DIR ${LIBDIR}/hiprt

  CONFIGURE_COMMAND
    ${HIPRT_WORKAROUND}
    PYTHON_BIN=${PYTHON_BINARY}
    ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${LIBDIR}/hiprt
    -G ${PLATFORM_ALT_GENERATOR}
    -S ${HIPRT_SOURCE_DIR}
    -B ${HIPRT_BUILD_DIR}
    ${DEFAULT_CMAKE_ARGS} ${HIPRT_EXTRA_ARGS}
)

if(WIN32)
  # Strip version from shared library name.
  ExternalProject_Add_Step(external_hiprt after_install
    COMMAND ${CMAKE_COMMAND} -E rename
      ${LIBDIR}/hiprt/bin/hiprt${HIPRT_LIBRARY_VERSION}64.dll ${LIBDIR}/hiprt/bin/hiprt64.dll
    DEPENDEES install
  )
  ExternalProject_Add_Step(external_hiprt after_install
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${LIBDIR}/hiprt
      ${HARVEST_TARGET}/hiprt
    DEPENDEES install
  )
else()
  # Strip version from shared library name.
  ExternalProject_Add_Step(external_hiprt after_install
    COMMAND ${CMAKE_COMMAND} -E rename
      ${LIBDIR}/hiprt/bin/${LIBPREFIX}hiprt${HIPRT_LIBRARY_VERSION}64.so ${LIBDIR}/hiprt/bin/${LIBPREFIX}hiprt64.so
    DEPENDEES install
  )
endif()
