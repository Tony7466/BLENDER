# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright 2006 Blender Foundation

# -----------------------------------------------------------------------------
# Shared Thumbnail Extraction Logic

include_directories(
  ../blenlib
  ../makesdna
  ../../../intern/guardedalloc
)

include_directories(
  SYSTEM
  ${ZLIB_INCLUDE_DIRS}
)

set(SRC
  src/blendthumb.hh
  src/blendthumb_extract.cc
  src/blendthumb_png.cc
)

set(LIB
  bf_blenlib
)

if(WIN32)
  # -----------------------------------------------------------------------------
  # Build `BlendThumb.dll`

  set(SRC_WIN32
    src/blendthumb_win32.cc
    src/blendthumb_win32.def
    src/blendthumb_win32.rc
    src/blendthumb_win32_dll.cc
  )

  add_definitions(-DNOMINMAX)

  add_library(BlendThumb SHARED ${SRC} ${SRC_WIN32})

  list(APPEND LIB
    dbghelp.lib
    Version.lib
  )
  target_link_libraries(BlendThumb ${LIB})
  set_target_properties(BlendThumb PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt")
endif()
if(UNIX AND NOT APPLE)
  # -----------------------------------------------------------------------------
  # Build `blender-thumbnailer` executable

  set(SRC_CMD
    src/blender_thumbnailer.cc
  )

  add_executable(blender-thumbnailer ${SRC} ${SRC_CMD})
  setup_platform_linker_flags(blender-thumbnailer)
  target_link_libraries(blender-thumbnailer ${LIB})
  if(DEFINED PTHREADS_LIBRARIES)
    target_link_libraries(blender-thumbnailer ${PTHREADS_LIBRARIES})
  endif()
endif()
if(APPLE)
  # -----------------------------------------------------------------------------
  # Build `blender-thumbnailer` appex

  set(SRC_CMD
    src/ThumbnailProvider.mm
    src/ThumbnailProvider.hh
    src/Info.plist
  )

  add_executable(blender-thumbnailer MACOSX_BUNDLE ${SRC} ${SRC_CMD})
  set_target_properties(blender-thumbnailer PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/src/Info.plist
    MACOSX_FRAMEWORK_IDENTIFIER org.blenderfoundation.blender.thumbnailer)
  # set_target_properties(blender-thumbnailer PROPERTIES
  #   XIB_RESOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/Base.lproj")
  # Doesn't affect other generators. So need to specify extension for others.
  set_target_properties(blender-thumbnailer PROPERTIES
    XCODE_PRODUCT_TYPE com.apple.product-type.app-extension)
  set_target_properties(blender-thumbnailer PROPERTIES
    BUNDLE_EXTENSION appex
    BL_CODESIGN_ENTITLEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ThumbnailExtensionMacOS.entitlements)
  setup_platform_linker_flags(blender-thumbnailer)
  set_target_properties(blender-thumbnailer PROPERTIES BUILD_WITH_INSTALL_RPATH true)
  list(APPEND LIB
    "-framework QuickLookThumbnailing"
    "-fobjc-link-runtime"
    "-fapplication-extension"
    "-e _NSExtensionMain"
  )
  target_link_libraries(blender-thumbnailer ${LIB})
  if(DEFINED PTHREADS_LIBRARIES)
    target_link_libraries(blender-thumbnailer ${PTHREADS_LIBRARIES})
  endif()
endif()
