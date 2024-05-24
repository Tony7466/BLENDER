# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Note the utility apps may use png/tiff/gif system libraries, but the
# library itself does not depend on them, so should give no problems.

set(HIPRT_CMAKE_FLAGS ${DEFAULT_CMAKE_FLAGS})

get_filename_component(_hip_path ${HIP_HIPCC_EXECUTABLE} DIRECTORY)
get_filename_component(_hip_path ${_hip_path} DIRECTORY)

if(WIN32)
  set(hiprt_configure HIP_PATH=${_hip_path} .\tools\premake5\win\premake5.exe vs2019)
  set(hiprt_build msbuild /m build/hiprt.sln /p:Configuration=Release)
else()
  set(hiprt_configure HIP_PATH=${_hip_path} ./tools/premake5/linux64/premake5 gmake)
  set(hiprt_build make -C build -j config=release_x64)
endif()

ExternalProject_Add(external_hiprt
  URL file://${PACKAGE_DIR}/${HIPRT_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${HIPRT_HASH_TYPE}=${HIPRT_HASH}
  PREFIX ${BUILD_DIR}/hiprt
  INSTALL_DIR ${LIBDIR}/hiprt

  PATCH_COMMAND
    ${PATCH_CMD} -p 1 -d ${BUILD_DIR}/hiprt/src/external_hiprt < ${PATCH_DIR}/hiprt.diff &&
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/orochi/src/external_orochi ${BUILD_DIR}/hiprt/src/external_hiprt/contrib/Orochi

  CONFIGURE_COMMAND
    cd ${BUILD_DIR}/hiprt/src/external_hiprt/ &&
    ${hiprt_configure} --bitcode=true --no-unittest=true --no-encrypt=true
  BUILD_COMMAND
    cd ${BUILD_DIR}/hiprt/src/external_hiprt/ &&
    ${hiprt_build}
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E copy ${BUILD_DIR}/hiprt/src/external_hiprt/dist/bin/Release/${LIBPREFIX}hiprt${HIPRT_LIBRARY_VERSION}64${SHAREDLIBEXT} ${LIBDIR}/hiprt/bin/${LIBPREFIX}hiprt64${SHAREDLIBEXT} &&
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/hiprt/src/external_hiprt/hiprt ${LIBDIR}/hiprt/include/hiprt &&
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/hiprt/src/external_hiprt/contrib/Orochi/ParallelPrimitives ${LIBDIR}/hiprt/include/orochi/ParallelPrimitives
)

add_dependencies(
  external_hiprt
  external_orochi
)

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
