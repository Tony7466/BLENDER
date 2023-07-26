# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenfont
  ../../blenkernel
  ../../blentranslation
  ../../depsgraph
  ../../gpu
  ../../imbuf
  ../../makesrna
  ../../windowmanager
  ../../bmesh
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  annotate_draw.cc
  annotate_paint.cc
  drawgpencil.cc
  editaction_gpencil.cc
  gpencil_add_blank.cc
  gpencil_add_lineart.cc
  gpencil_add_monkey.cc
  gpencil_add_stroke.cc
  gpencil_armature.cc
  gpencil_bake_animation.cc
  gpencil_convert.c
  gpencil_data.c
  gpencil_edit.c
  gpencil_edit_curve.c
  gpencil_fill.c
  gpencil_interpolate.c
  gpencil_merge.c
  gpencil_mesh.cc
  gpencil_ops.c
  gpencil_ops_versioning.c
  gpencil_paint.c
  gpencil_primitive.c
  gpencil_sculpt_paint.c
  gpencil_select.c
  gpencil_undo.cc
  gpencil_utils.c
  gpencil_uv.c
  gpencil_vertex_ops.c
  gpencil_vertex_paint.c
  gpencil_weight_paint.c

  gpencil_intern.h
  gpencil_trace.h
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
)

if(WITH_POTRACE)
  list(APPEND SRC
    gpencil_trace_ops.c
    gpencil_trace_utils.c
  )
  list(APPEND INC
    ${POTRACE_INCLUDE_DIRS}
  )
  list(APPEND LIB
    ${POTRACE_LIBRARIES}
  )
  add_definitions(-DWITH_POTRACE)
endif()


blender_add_lib(bf_editor_gpencil_legacy "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_gpencil_legacy bf_rna)
