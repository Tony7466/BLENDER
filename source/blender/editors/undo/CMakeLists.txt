# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../asset_system
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../bmesh
  ../../makesrna
  ../../windowmanager
)

set(INC_SYS
)

set(SRC
  ed_undo.cc
  memfile_undo.cc
  undo_system_types.cc

  undo_intern.hh
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_armature
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_curve
  PRIVATE bf_editor_curves
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_grease_pencil
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_lattice
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_metaball
  PRIVATE bf_editor_object
  PRIVATE bf_editor_physics
  PRIVATE bf_editor_render
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf_editor_space_image
  PRIVATE bf_editor_space_outliner
  PRIVATE bf_editor_space_text
  PRIVATE bf_editor_util
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

blender_add_lib(bf_editor_undo "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
