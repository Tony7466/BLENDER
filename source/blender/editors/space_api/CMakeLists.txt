# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../io
  ../../asset_system
  ../../blenkernel
  ../../gpu
  ../../makesrna
  ../../windowmanager
)

set(INC_SYS

)

set(SRC
  spacetypes.cc
)

set(LIB
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_armature
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_curve
  PRIVATE bf_editor_curves
  bf_editor_geometry
  PRIVATE bf_editor_gizmo_library
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_grease_pencil
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_io
  PRIVATE bf_editor_lattice
  PRIVATE bf_editor_mask
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_metaball
  PRIVATE bf_editor_object
  PRIVATE bf_editor_physics
  PRIVATE bf_editor_render
  PRIVATE bf_editor_scene
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf_editor_sound
  bf_editor_space_action
  bf_editor_space_buttons
  bf_editor_space_clip
  bf_editor_space_console
  bf_editor_space_file
  bf_editor_space_graph
  bf_editor_space_image
  bf_editor_space_info
  bf_editor_space_nla
  bf_editor_space_node
  bf_editor_space_outliner
  bf_editor_space_script
  bf_editor_space_sequencer
  bf_editor_space_spreadsheet
  bf_editor_space_statusbar
  bf_editor_space_text
  bf_editor_space_topbar
  bf_editor_space_userpref
  bf_editor_space_view3d
  PRIVATE bf_editor_transform
  PRIVATE bf_editor_util
  PRIVATE bf_editor_uvedit
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
)

blender_add_lib(bf_editor_space_api "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
