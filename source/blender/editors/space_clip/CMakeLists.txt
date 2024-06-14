# SPDX-FileCopyrightText: 2011 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../windowmanager

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  clip_buttons.cc
  clip_dopesheet_draw.cc
  clip_dopesheet_ops.cc
  clip_draw.cc
  clip_editor.cc
  clip_graph_draw.cc
  clip_graph_ops.cc
  clip_ops.cc
  clip_utils.cc
  space_clip.cc
  tracking_ops.cc
  tracking_ops_detect.cc
  tracking_ops_orient.cc
  tracking_ops_plane.cc
  tracking_ops_solve.cc
  tracking_ops_stabilize.cc
  tracking_ops_track.cc
  tracking_ops_utils.cc
  tracking_select.cc

  clip_intern.hh
  tracking_ops_intern.hh
)

set(LIB
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mask
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_util
  PRIVATE bf_editor_uvedit
  PRIVATE bf_gpu
  PRIVATE bf_imbuf
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_space_clip "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_space_clip bf_rna)
