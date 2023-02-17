# SPDX-License-Identifier: Apache-2.0
# Copyright 2011-2022 Blender Foundation

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
  ${CMAKE_BINARY_DIR}/source/blender/makesrna/intern
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
  finalEngine.h
  finalEngine.cc
  viewportEngine.h
  viewportEngine.cc
  camera.h
  camera.cc
  utils.h
  utils.cc

  renderTaskDelegate.cc
  renderTaskDelegate.h

  sceneDelegate/blenderSceneDelegate.h
  sceneDelegate/blenderSceneDelegate.cc
  sceneDelegate/object.h
  sceneDelegate/object.cc
  sceneDelegate/material.h
  sceneDelegate/material.cc
  sceneDelegate/world.h
  sceneDelegate/world.cc
)

set(LIB
  ${Epoxy_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${BOOST_LIBRARIES}
)

blender_add_lib(bf_render_hydra "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

add_dependencies(bf_render_hydra bf_rna)
