# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../asset_system
  ../../blenfont
  ../../blenloader
  ../../blentranslation
  ../../depsgraph
  ../../gpu
  ../../windowmanager
  ../../../../intern/clog
  ../../../../intern/eigen
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
  editarmature_undo.cc
  meshlaplacian.cc
  pose_edit.cc
  pose_group.cc
  pose_lib_2.cc
  pose_select.cc
  pose_slide.cc
  pose_transform.cc
  pose_utils.cc

  armature_intern.h
  meshlaplacian.h
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::rna
)


blender_add_lib(bf_editor_armature "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

