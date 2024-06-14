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
  ../../draw
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../render
  ../../windowmanager
  ../../../../intern/mantaflow/extern

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  drawobject.cc
  space_view3d.cc
  view3d_buttons.cc
  view3d_camera_control.cc
  view3d_context.cc
  view3d_cursor_snap.cc
  view3d_draw.cc
  view3d_edit.cc
  view3d_gizmo_armature.cc
  view3d_gizmo_camera.cc
  view3d_gizmo_empty.cc
  view3d_gizmo_forcefield.cc
  view3d_gizmo_light.cc
  view3d_gizmo_navigate.cc
  view3d_gizmo_navigate_type.cc
  view3d_gizmo_preselect.cc
  view3d_gizmo_preselect_type.cc
  view3d_gizmo_ruler.cc
  view3d_gizmo_tool_generic.cc
  view3d_header.cc
  view3d_iterators.cc
  view3d_navigate.cc
  view3d_navigate_fly.cc
  view3d_navigate_smoothview.cc
  view3d_navigate_view_all.cc
  view3d_navigate_view_axis.cc
  view3d_navigate_view_camera.cc
  view3d_navigate_view_center_cursor.cc
  view3d_navigate_view_center_pick.cc
  view3d_navigate_view_dolly.cc
  view3d_navigate_view_move.cc
  view3d_navigate_view_ndof.cc
  view3d_navigate_view_orbit.cc
  view3d_navigate_view_pan.cc
  view3d_navigate_view_roll.cc
  view3d_navigate_view_rotate.cc
  view3d_navigate_view_zoom.cc
  view3d_navigate_walk.cc
  view3d_navigate_zoom_border.cc
  view3d_ops.cc
  view3d_placement.cc
  view3d_project.cc
  view3d_select.cc
  view3d_snap.cc
  view3d_utils.cc
  view3d_view.cc

  view3d_intern.hh
  view3d_navigate.hh
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf_asset_system
  PRIVATE bf::blenfont
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf_bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_draw
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_armature
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_curve
  bf_editor_curves
  PRIVATE bf_editor_geometry
  PRIVATE bf_editor_gizmo_library
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_grease_pencil
  PRIVATE bf_editor_interface
  bf_editor_lattice
  bf_editor_mesh
  PRIVATE bf_editor_metaball
  PRIVATE bf_editor_object
  PRIVATE bf_editor_physics
  PRIVATE bf_editor_render
  PRIVATE bf_editor_scene
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_info
  PRIVATE bf_editor_space_outliner
  PRIVATE bf_editor_transform
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf_gpu
  PRIVATE bf_imbuf
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_python_ext
  PRIVATE bf_render
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

if(WITH_PYTHON)
  list(APPEND INC ../../python)
  add_definitions(-DWITH_PYTHON)
endif()

if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

if(WITH_XR_OPENXR)
  add_definitions(-DWITH_XR_OPENXR)
endif()

blender_add_lib(bf_editor_space_view3d "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_space_view3d bf_rna)
