# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blentranslation
  ../../bmesh
  ../../geometry
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
  uvedit_buttons.cc
  uvedit_clipboard.cc
  uvedit_clipboard_graph_iso.cc
  uvedit_draw.cc
  uvedit_islands.cc
  uvedit_ops.cc
  uvedit_path.cc
  uvedit_rip.cc
  uvedit_select.cc
  uvedit_smart_stitch.cc
  uvedit_unwrap_ops.cc

  uvedit_clipboard_graph_iso.hh
  uvedit_intern.hh
)

set(LIB
  PRIVATE bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blentranslation
  bf_bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_object
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_space_image
  PRIVATE bf_editor_space_node
  PRIVATE bf_editor_space_view3d
  PRIVATE bf_editor_transform
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf_geometry
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_nodes_shader
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


blender_add_lib(bf_editor_uvedit "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_uvedit bf_rna)
