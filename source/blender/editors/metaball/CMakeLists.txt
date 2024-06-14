# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../render
  ../../windowmanager
)

set(INC_SYS

)

set(SRC
  editmball_undo.cc
  mball_edit.cc
  mball_ops.cc

  mball_intern.hh
)

set(LIB
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

blender_add_lib(bf_editor_metaball "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
