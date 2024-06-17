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
  ../../geometry
  ../../gpencil_modifiers_legacy
  ../../gpu
  ../../ikplugin
  ../../imbuf
  ../../makesrna
  ../../modifiers
  ../../python
  ../../render
  ../../shader_fx
  ../../windowmanager

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  add_modifier_assets.cc
  object_add.cc
  object_bake.cc
  object_bake_api.cc
  object_bake_simulation.cc
  object_collection.cc
  object_constraint.cc
  object_data_transfer.cc
  object_data_transform.cc
  object_edit.cc
  object_gpencil_modifier.cc
  object_hook.cc
  object_light_linking_ops.cc
  object_modes.cc
  object_modifier.cc
  object_ops.cc
  object_random.cc
  object_relations.cc
  object_remesh.cc
  object_select.cc
  object_shader_fx.cc
  object_shapekey.cc
  object_transform.cc
  object_utils.cc
  object_vgroup.cc
  object_volume.cc
  object_warp.cc

  object_intern.hh
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf_asset_system
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf_bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_armature
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_curve
  PRIVATE bf_editor_curves
  PRIVATE bf_editor_gpencil_legacy
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_lattice
  bf_editor_mesh
  bf_editor_metaball
  bf_editor_grease_pencil
  PRIVATE bf_editor_physics
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_image
  PRIVATE bf_editor_space_outliner
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf_editor_uvedit
  PRIVATE bf::extern::fmtlib
  PRIVATE bf_geometry
  PRIVATE bf_gpencil_modifiers_legacy
  PRIVATE bf_gpu
  PRIVATE bf_ikplugin
  PRIVATE bf_imbuf
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_modifiers
  PRIVATE bf_nodes
  PRIVATE bf_python
  bf_render
  PRIVATE bf_rna
  bf_windowmanager
)


if(WITH_PYTHON)
  add_definitions(-DWITH_PYTHON)
endif()

if(WITH_EXPERIMENTAL_FEATURES)
  add_definitions(-DWITH_SIMULATION_DATABLOCK)
  add_definitions(-DWITH_POINT_CLOUD)
endif()

blender_add_lib(bf_editor_object "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_object bf_rna)
