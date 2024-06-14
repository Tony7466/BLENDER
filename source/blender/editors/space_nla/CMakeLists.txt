# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../windowmanager

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  nla_buttons.cc
  nla_draw.cc
  nla_edit.cc
  nla_ops.cc
  nla_select.cc
  nla_tracks.cc
  space_nla.cc

  nla_intern.hh
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_action
  PRIVATE bf_editor_space_api
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

blender_add_lib(bf_editor_space_nla "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h dna_type_offsets.h
add_dependencies(bf_editor_space_nla bf_rna)
