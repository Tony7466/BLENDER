# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../asset_system
  ../../blenkernel
  ../../blenloader
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../windowmanager
  ../../../../intern/eigen
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  armature_add.cc
  armature_edit.cc
  armature_naming.cc
  armature_ops.cc
  armature_relations.cc
  armature_select.cc
  armature_skinning.cc
  armature_utils.cc
  bone_collections.cc
  editarmature_undo.cc
  meshlaplacian.cc
  pose_edit.cc
  pose_lib_2.cc
  pose_select.cc
  pose_slide.cc
  pose_transform.cc
  pose_utils.cc

  armature_intern.hh
  meshlaplacian.h
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf_asset_system
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_outliner
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf::intern::clog
  PRIVATE bf_intern_eigen
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_modifiers
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_armature "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_armature bf_rna)
