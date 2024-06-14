# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../windowmanager
  ../../../../intern/mantaflow/extern
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  dynamicpaint_ops.cc
  particle_boids.cc
  particle_edit.cc
  particle_edit_undo.cc
  particle_object.cc
  physics_fluid.cc
  physics_ops.cc
  physics_pointcache.cc
  rigidbody_constraint.cc
  rigidbody_object.cc
  rigidbody_world.cc

  particle_edit_utildefines.h
  physics_intern.hh
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf_gpu
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_intern_rigidbody
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

if(WITH_MOD_FLUID)
  list(APPEND LIB
    bf_intern_mantaflow
  )
  add_definitions(-DWITH_FLUID)
endif()

if(WITH_BULLET)
  list(APPEND INC
    ../../../../intern/rigidbody
  )
  add_definitions(-DWITH_BULLET)
endif()


blender_add_lib(bf_editor_physics "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_physics bf_rna)
