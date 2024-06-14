# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../asset_system
  ../../blenkernel
  ../../blentranslation
  ../../bmesh
  ../../functions
  ../../makesrna
  ../../nodes
  ../../windowmanager

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  geometry_attributes.cc
  geometry_ops.cc
  geometry_randomization.cc
  node_group_operator.cc

  geometry_intern.hh
)

set(LIB
  PRIVATE bf_asset_system
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf_bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mesh
  bf_editor_object
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_nodes
  PRIVATE bf_rna
  bf_windowmanager
)

blender_add_lib(bf_editor_geometry "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h dna_type_offsets.h
add_dependencies(bf_editor_geometry bf_rna)
