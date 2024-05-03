# SPDX-FileCopyrightText: 2017-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(SNDFILE_EXTRA_ARGS
  -DCMAKE_POLICY_DEFAULT_CMP0074=NEW
  -DOgg_ROOT=${LIBDIR}/ogg
  -DVorbis_ROOT=${LIBDIR}/vorbis
  -DOpus_ROOT=${LIBDIR}/opus
  -DFLAC_ROOT=${LIBDIR}/flac
  -DCMAKE_DISABLE_FIND_PACKAGE_ALSA=ON
  -DCMAKE_DISABLE_FIND_PACKAGE_mpg123=ON
  -DCMAKE_DISABLE_FIND_PACKAGE_Speex=ON
  -DCMAKE_DISABLE_FIND_PACKAGE_SQLite3=ON
  -DBUILD_PROGRAMS=OFF
  -DBUILD_EXAMPLES=OFF
  -DBUILD_TESTING=OFF
  -DPYTHON_EXECUTABLE=${PYTHON_BINARY}
)

if(WIN32)
  # We set FLAC__NO_DLL, otherwise we cannot statically link FLAC
  set(SNDFILE_C_FLAGS "${CMAKE_C_FLAGS} -DFLAC__NO_DLL=1")
  set(SNDFILE_EXTRA_ARGS
    ${SNDFILE_EXTRA_ARGS}
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_C_FLAGS=${SNDFILE_C_FLAGS}
    -DLame_ROOT=${LIBDIR}/lame
    -DLAME_INCLUDE_DIR=${LIBDIR}/lame/include/
  )
  set(SNDFILE_PATCH ${CMAKE_COMMAND} -E copy
    ${PATCH_DIR}/cmake/modules/FindLame.cmake
    ${BUILD_DIR}/sndfile/src/external_sndfile/cmake/FindLame.cmake
  )
else()
  set(SNDFILE_PATCH)
  set(SNDFILE_EXTRA_ARGS
    ${SNDFILE_EXTRA_ARGS}
    -DBUILD_SHARED_LIBS=OFF
    -DLAME_ROOT=${LIBDIR}/lame
    -DMP3LAME_INCLUDE_DIR=${LIBDIR}/lame/include
    -DMP3LAME_LIBRARY=${LIBDIR}/lame/lib/libmp3lame.a
  )
endif()

ExternalProject_Add(external_sndfile
  URL file://${PACKAGE_DIR}/${SNDFILE_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${SNDFILE_HASH_TYPE}=${SNDFILE_HASH}
  PREFIX ${BUILD_DIR}/sndfile

  PATCH_COMMAND ${SNDFILE_PATCH}

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${LIBDIR}/sndfile
    ${DEFAULT_CMAKE_FLAGS}
    ${SNDFILE_EXTRA_ARGS}

  INSTALL_DIR ${LIBDIR}/sndfile
)

if(BUILD_MODE STREQUAL Release AND WIN32)
  ExternalProject_Add_Step(external_sndfile after_install
    COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBDIR}/sndfile/bin/sndfile.dll
      ${HARVEST_TARGET}/sndfile/lib/sndfile.dll
    COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBDIR}/sndfile/lib/sndfile.lib
      ${HARVEST_TARGET}/sndfile/lib/sndfile.lib
    COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBDIR}/sndfile/include/sndfile.h
      ${HARVEST_TARGET}/sndfile/include/sndfile.h

    DEPENDEES install
  )
endif()

add_dependencies(
  external_sndfile
  external_ogg
  external_vorbis
  external_opus
  external_flac
  external_lame
)
