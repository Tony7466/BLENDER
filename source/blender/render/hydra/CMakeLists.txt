# SPDX-FileCopyrightText: 2011-2022 Blender Foundation
#
# SPDX-License-Identifier: Apache-2.0

# This suppresses the warning "This file includes at least one deprecated or antiquated
# header which may be removed without further notice at a future date", which is caused
# by the USD library including <ext/hash_set> on Linux. This has been reported at:
# https://github.com/PixarAnimationStudios/USD/issues/1057.
if(UNIX AND NOT APPLE)
  add_definitions(-D_GLIBCXX_PERMIT_BACKWARD_HASH)
endif()
if(WIN32)
  add_definitions(-DNOMINMAX -DWIN32_LEAN_AND_MEAN -DBOOST_DEBUG_PYTHON)
endif()
add_definitions(-DBOOST_ALL_NO_LIB)

# Precompiled Linux libs are made with GCC, and USD uses some extensions
# which lead to an incompatible ABI for Clang. Using those extensions with
# Clang as well works around the issue.
if(UNIX AND NOT APPLE)
  if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    if(EXISTS ${LIBDIR})
      add_definitions(-DARCH_HAS_GNU_STL_EXTENSIONS)
    endif()
  endif()
endif()

# USD headers use deprecated TBB headers, silence warning.
add_definitions(-DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)

if(WIN32)
  # Some USD library headers trigger the "unreferenced formal parameter"
  # warning alert.
  # Silence them by restore warn C4100 back to w4
  remove_cc_flag("/w34100")
endif()

set(INC
  ../../../../intern/clog
  ../../../../intern/glew-mx
  ../../../../intern/guardedalloc
  ../../makesdna
  ../../makesrna
  ../../nodes
  ../../blenlib
  ../../depsgraph
  ../../blenkernel
  ../../imbuf
  ../../io/usd
  ../../gpu
  ../../gpu/intern
  ../../python/intern
  ..
  ${CMAKE_BINARY_DIR}/source/blender/makesrna/intern
)

set(INC_SYS
  ${PYTHON_INCLUDE_DIRS}
  ${Epoxy_INCLUDE_DIRS}
  ${USD_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
)

set(LIB
  ${Epoxy_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${USD_LIBRARIES}
  ${TBB_LIBRARIES}
)

if(WITH_MATERIALX)
  add_definitions(-DWITH_MATERIALX)
  list(APPEND LIB MaterialXCore)
endif()

set(SRC
  python.cc

  camera.h
  camera.cc
  engine.h
  engine.cc
  final_engine.h
  final_engine.cc
  preview_engine.h
  preview_engine.cc
  viewport_engine.h
  viewport_engine.cc

  render_task_delegate.cc
  render_task_delegate.h
  light_tasks_delegate.cc
  light_tasks_delegate.h

  scene_delegate/blender_scene_delegate.cc
  scene_delegate/blender_scene_delegate.h
  scene_delegate/curves.cc
  scene_delegate/curves.h
  scene_delegate/id.cc
  scene_delegate/id.h
  scene_delegate/image.cc
  scene_delegate/image.h
  scene_delegate/instancer.cc
  scene_delegate/instancer.h
  scene_delegate/light.cc
  scene_delegate/light.h
  scene_delegate/material.cc
  scene_delegate/material.h
  scene_delegate/mesh.cc
  scene_delegate/mesh.h
  scene_delegate/mtlx_hydra_adapter.cc
  scene_delegate/mtlx_hydra_adapter.h
  scene_delegate/object.cc
  scene_delegate/object.h
  scene_delegate/settings.h
  scene_delegate/usd_scene_delegate.cc
  scene_delegate/usd_scene_delegate.hh
  scene_delegate/volume.cc
  scene_delegate/volume.h
  scene_delegate/volume_modifier.cc
  scene_delegate/volume_modifier.h
  scene_delegate/world.cc
  scene_delegate/world.h
)

blender_add_lib(bf_render_hydra "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

add_dependencies(bf_render_hydra bf_rna)
