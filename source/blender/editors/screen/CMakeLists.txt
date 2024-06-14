# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../asset
  ../include
  ../../asset_system
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../bmesh
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../sequencer
  ../../windowmanager
  ../../../../extern/fmtlib/include
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  area.cc
  area_query.cc
  area_utils.cc
  glutil.cc
  screen_context.cc
  screen_draw.cc
  screen_edit.cc
  screen_geometry.cc
  screen_ops.cc
  screen_user_menu.cc
  screendump.cc
  workspace_edit.cc
  workspace_layout_edit.cc
  workspace_listen.cc

  screen_intern.hh
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf::blenfont
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_armature
  PRIVATE bf_editor_asset
  bf_editor_datafiles
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_object
  PRIVATE bf_editor_scene
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_buttons
  PRIVATE bf_editor_space_clip
  PRIVATE bf_editor_space_file
  PRIVATE bf_editor_space_graph
  PRIVATE bf_editor_space_image
  PRIVATE bf_editor_space_nla
  PRIVATE bf_editor_space_node
  bf_editor_space_sequencer
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_util
  PRIVATE bf_gpu
  PRIVATE bf_imbuf
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::extern::fmtlib
  PRIVATE bf_rna
  PRIVATE bf_sequencer
  PRIVATE bf_windowmanager
)

if(WITH_HEADLESS)
  add_definitions(-DWITH_HEADLESS)
endif()

blender_add_lib(bf_editor_screen "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_screen bf_rna)
