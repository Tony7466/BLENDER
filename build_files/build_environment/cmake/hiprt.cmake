# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Note the utility apps may use png/tiff/gif system libraries, but the
# library itself does not depend on them, so should give no problems.

set(HIPRT_CMAKE_FLAGS ${DEFAULT_CMAKE_FLAGS})

# TODO:
# Do anything with these?
# -DHIPRT_EXPORTS=ON
# -D__USE_HIP__=ON
# -DHIPRT_BITCODE_LINKING=ON
# -DHIPRT_LOAD_FROM_STRING=OFF
# -DORO_PRECOMPILED=ON

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

  # TODO: should not be downloading git repos here
  PATCH_COMMAND ${PATCH_CMD} -p 1 -d
    ${BUILD_DIR}/hiprt/src/external_hiprt <
    ${PATCH_DIR}/hiprt.diff &&
    cd ${BUILD_DIR}/hiprt/src/external_hiprt/contrib &&
    git clone git@github.com:amdadvtech/Orochi.git &&
    cd Orochi && git checkout c82a229f5a424117855b86b78b480d003419bf66 && cd .. &&
    git clone https://github.com/amdadvtech/easy-encryption

  # TODO: disabling unittest is not working, binary still gets written
  CONFIGURE_COMMAND
    cd ${BUILD_DIR}/hiprt/src/external_hiprt/ &&
    ${hiprt_configure} --bitcode=true --unittest=false
  BUILD_COMMAND
    cd ${BUILD_DIR}/hiprt/src/external_hiprt/ &&
    ${hiprt_build}
  # TODO: does premake not have an install command?
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/hiprt/src/external_hiprt/dist/bin/Release ${LIBDIR}/hiprt/bin &&
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/hiprt/src/external_hiprt/hiprt ${LIBDIR}/hiprt/include/hiprt &&
    ${CMAKE_COMMAND} -E copy_directory ${BUILD_DIR}/hiprt/src/external_hiprt/contrib/Orochi/ParallelPrimitives ${LIBDIR}/hiprt/include/orochi/ParallelPrimitives
)

if(WIN32)
  if(BUILD_MODE STREQUAL Release)
    ExternalProject_Add_Step(external_hiprt after_install
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${LIBDIR}/hiprt/include
        ${HARVEST_TARGET}/hiprt/include
      COMMAND ${CMAKE_COMMAND} -E copy
        ${LIBDIR}/hiprt/bin/hiprt*64.dll
        ${HARVEST_TARGET}/hiprt/bin/
      DEPENDEES install
    )
  else()
    ExternalProject_Add_Step(external_hiprt after_install
      COMMAND ${CMAKE_COMMAND} -E copy
        ${LIBDIR}/hiprt/bin/hiprt*64D.dll
        ${HARVEST_TARGET}/hiprt/bin/
      DEPENDEES install
    )
  endif()
endif()
