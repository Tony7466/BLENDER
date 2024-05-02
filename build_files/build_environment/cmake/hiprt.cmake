# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Note the utility apps may use png/tiff/gif system libraries, but the
# library itself does not depend on them, so should give no problems.

set(HIPRT_CMAKE_FLAGS ${DEFAULT_CMAKE_FLAGS})

set(HIPRT_EXTRA_ARGS
  -DHIPRT_EXPORTS=ON
  -D__USE_HIP__=ON
  -DHIPRT_BITCODE_LINKING=ON
  -DHIPRT_LOAD_FROM_STRING=OFF
  -DORO_PRECOMPILED=ON
)

if(WIN32)
  set(HIPRT_EXTRA_ARGS
    ${HIPRT_EXTRA_ARGS}
    -DCMAKE_DEBUG_POSTFIX=_d
  )
endif()


ExternalProject_Add(external_hiprt
  URL file://${PACKAGE_DIR}/${HIPRT_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${HIPRT_HASH_TYPE}=${HIPRT_HASH}
  CMAKE_GENERATOR ${PLATFORM_ALT_GENERATOR}
  PREFIX ${BUILD_DIR}/hiprt

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${LIBDIR}/hiprt
    ${HIPRT_CMAKE_FLAGS}
    ${HIPRT_EXTRA_ARGS}

  INSTALL_DIR ${LIBDIR}/hiprt
)

if(WIN32)
  if(BUILD_MODE STREQUAL Release)
    ExternalProject_Add_Step(external_hiprt after_install
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${LIBDIR}/hiprt/hiprt
        ${HARVEST_TARGET}/hiprt/hiprt
      COMMAND ${CMAKE_COMMAND} -E copy
        ${LIBDIR}/hiprt/dist/bin/Release/hiprt*64.dll
        ${HARVEST_TARGET}/hiprt/bin/hiprt*64.dll

      DEPENDEES install
    )
  else()
    ExternalProject_Add_Step(external_hiprt after_install
      COMMAND ${CMAKE_COMMAND} -E copy
        ${LIBDIR}/hiprt/dist/bin/Debug/hiprt*64D.dll
        ${HARVEST_TARGET}/hiprt/bin/hiprt*64D.dll

      DEPENDEES install
    )
  endif()
endif()
