# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../makesrna
  ../../sequencer
  ../../windowmanager
)

set(INC_SYS

)

set(SRC
  scene_edit.cc
  scene_fps.cc
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_render
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_util
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_sequencer
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_scene "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
