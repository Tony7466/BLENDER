# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../gpu
  ../../makesrna
  ../../windowmanager
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  curve_ops.cc
  editcurve.cc
  editcurve_add.cc
  editcurve_paint.cc
  editcurve_pen.cc
  editcurve_query.cc
  editcurve_select.cc
  editcurve_undo.cc
  editfont.cc
  editfont_undo.cc

  curve_intern.hh
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_outliner
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_transform
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf::extern::curve_fit_nd
  PRIVATE bf_gpu
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_curve "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_curve bf_rna)
