# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../bmesh
  ../../gpu
  ../../makesrna
  ../../windowmanager

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  buttons_context.cc
  buttons_ops.cc
  buttons_texture.cc
  space_buttons.cc

  buttons_intern.hh
)

set(LIB
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::dna
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_physics
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_node
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

if(WITH_EXPERIMENTAL_FEATURES)
  add_definitions(-DWITH_SIMULATION_DATABLOCK)
  add_definitions(-DWITH_POINT_CLOUD)
endif()

blender_add_lib(bf_editor_space_buttons "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h dna_type_offsets.h
add_dependencies(bf_editor_space_buttons bf_rna)
