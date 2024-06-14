# SPDX-FileCopyrightText: 2012 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../windowmanager
)

set(INC_SYS
)

set(SRC
  mask_add.cc
  mask_draw.cc
  mask_edit.cc
  mask_editaction.cc
  mask_ops.cc
  mask_query.cc
  mask_relationships.cc
  mask_select.cc
  mask_shapekey.cc

  mask_intern.hh
)

set(LIB
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_clip
  PRIVATE bf_editor_space_image
  PRIVATE bf_editor_space_sequencer
  PRIVATE bf_editor_util
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::animrig
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_mask "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
