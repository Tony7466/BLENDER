# SPDX-License-Identifier: Apache-2.0
# Copyright 2011-2022 Blender Foundation

if(NOT TARGET MaterialXCore OR NOT TARGET MaterialXFormat)
  message(FATAL_ERROR "Hydra Scene Delegate requires MaterialX")
endif()

if(WIN32)
  add_definitions(-DNOMINMAX -DWIN32_LEAN_AND_MEAN -DBOOST_DEBUG_PYTHON)
endif()

# USD headers use deprecated TBB headers, silence warning.
add_definitions(-DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)

add_definitions(${GFLAGS_DEFINES})
add_definitions(${GLOG_DEFINES})
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
add_definitions(-DBLENDER_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

add_definitions(-DBOOST_ALL_NO_LIB)

set(INC
  ../../../../intern/glew-mx
  ../../../../intern/guardedalloc
  ../../makesdna
  ../../makesrna
  ../../nodes
  ../../blenlib
  ../../depsgraph
  ../../blenkernel
  ../../io/usd
  ../../io/common
  ../../io/usd/intern
  ../../gpu
  ../../gpu/opengl
  ../../gpu/intern
  ..
)

set(INC_SYS
  ${Epoxy_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${USD_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
)

set(SRC
  python.cc

  engine.h
  engine.cc
  final_engine.h
  final_engine.cc
  preview_engine.h
  preview_engine.cc
  viewport_engine.h
  viewport_engine.cc
  camera.h
  camera.cc
  utils.h
  utils.cc

  render_task_delegate.cc
  render_task_delegate.h
  simple_light_task_delegate.cc
  simple_light_task_delegate.h

  scene_delegate/blender_scene_delegate.h
  scene_delegate/blender_scene_delegate.cc
  scene_delegate/id.h
  scene_delegate/id.cc
  scene_delegate/object.h
  scene_delegate/object.cc
  scene_delegate/material.h
  scene_delegate/material.cc
  scene_delegate/mesh.h
  scene_delegate/mesh.cc
  scene_delegate/mtlx_hydra_adapter.h
  scene_delegate/mtlx_hydra_adapter.cc
  scene_delegate/light.h
  scene_delegate/light.cc
  scene_delegate/world.h
  scene_delegate/world.cc
)

set(LIB
  ${Epoxy_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${BOOST_LIBRARIES}
)

blender_add_lib(bf_render_hydra "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

target_link_libraries(bf_render_hydra PRIVATE
    MaterialXCore
    MaterialXFormat)

add_dependencies(bf_render_hydra bf_rna)
