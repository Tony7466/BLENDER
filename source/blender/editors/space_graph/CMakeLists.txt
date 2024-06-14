# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blenloader
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
  graph_buttons.cc
  graph_draw.cc
  graph_edit.cc
  graph_ops.cc
  graph_select.cc
  graph_slider_ops.cc
  graph_utils.cc
  graph_view.cc
  space_graph.cc

  graph_intern.hh
)

set(LIB
  PRIVATE audaspace
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf_editor_animation
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_api
  PRIVATE bf_editor_undo
  PRIVATE bf_editor_util
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::animrig
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

if(WITH_AUDASPACE)
  list(APPEND LIB
    bf_intern_audaspace
  )
  list(APPEND INC_SYS
    ${AUDASPACE_C_INCLUDE_DIRS}
  )
  if(WITH_SYSTEM_AUDASPACE)
    list(APPEND LIB
      ${AUDASPACE_C_LIBRARIES}
      ${AUDASPACE_PY_LIBRARIES}
    )
  endif()
  add_definitions(-DWITH_AUDASPACE)
endif()


blender_add_lib(bf_editor_space_graph "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h dna_type_offsets.h
add_dependencies(bf_editor_space_graph bf_rna)
