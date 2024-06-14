# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../asset
  ../include
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

  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  image_buttons.cc
  image_draw.cc
  image_edit.cc
  image_ops.cc
  image_sequence.cc
  image_undo.cc
  space_image.cc

  image_intern.hh
)

set(LIB
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf_bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_draw
  PRIVATE bf_editor_asset
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_mask
  PRIVATE bf_editor_mesh
  PRIVATE bf_editor_object
  PRIVATE bf_editor_render
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_sculpt_paint
  PRIVATE bf_editor_space_node
  PRIVATE bf_editor_transform
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  bf_editor_uvedit
  PRIVATE bf_gpu
  PRIVATE bf_imbuf
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_render
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

if(WITH_IMAGE_OPENJPEG)
  add_definitions(-DWITH_OPENJPEG)
endif()

if(WITH_IMAGE_OPENEXR)
  add_definitions(-DWITH_OPENEXR)
endif()

if(WITH_IMAGE_CINEON)
  add_definitions(-DWITH_CINEON)
endif()

if(WITH_IMAGE_WEBP)
  add_definitions(-DWITH_WEBP)
endif()

blender_add_lib(bf_editor_space_image "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h dna_type_offsets.h
add_dependencies(bf_editor_space_image bf_rna)
