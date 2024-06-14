# SPDX-FileCopyrightText: 2006-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../bmesh
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../windowmanager
  ../../../../extern/curve_fit_nd
  ../../geometry
  ../sculpt_paint
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  intern/grease_pencil_add.cc
  intern/grease_pencil_cutter.cc
  intern/grease_pencil_edit.cc
  intern/grease_pencil_frames.cc
  intern/grease_pencil_geom.cc
  intern/grease_pencil_image_render.cc
  intern/grease_pencil_layers.cc
  intern/grease_pencil_material.cc
  intern/grease_pencil_ops.cc
  intern/grease_pencil_primitive.cc
  intern/grease_pencil_select.cc
  intern/grease_pencil_undo.cc
  intern/grease_pencil_utils.cc
  intern/grease_pencil_weight_paint.cc
)

set(LIB
  bf_blenkernel
  PRIVATE bf::animrig
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_curves
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_undo
  PRIVATE bf_geometry
  PRIVATE bf_gpu
  PRIVATE bf_imbuf
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::intern::clog
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
  extern_curve_fit_nd
  extern_fmtlib
)

blender_add_lib(bf_editor_grease_pencil "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_dependencies(bf_editor_curves bf_rna)
