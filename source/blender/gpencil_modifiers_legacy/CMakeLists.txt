# SPDX-FileCopyrightText: 2018 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  intern
  ../blenkernel
  ../blentranslation
  ../bmesh
  ../editors/include
  ../makesrna
  ../render
  ../render/intern
  ../windowmanager
  ../../../intern/eigen
  ../geometry

  # RNA_prototypes.hh
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
  ${ZLIB_INCLUDE_DIRS}
)

set(SRC
  # Lineart code
  intern/lineart/lineart_chain.cc
  intern/lineart/lineart_cpu.cc
  intern/lineart/lineart_ops.cc
  intern/lineart/lineart_shadow.cc
  intern/lineart/lineart_util.cc

  intern/lineart/MOD_lineart.h
  intern/lineart/lineart_intern.h
)

if(WITH_TBB)
add_definitions(-DWITH_TBB)
if(WIN32)
  # TBB includes Windows.h which will define min/max macros
  # that will collide with the stl versions.
  add_definitions(-DNOMINMAX)
endif()
list(APPEND INC_SYS
  ${TBB_INCLUDE_DIRS}
)

list(APPEND LIB
  ${TBB_LIBRARIES}
)
endif()

set(LIB
  PRIVATE bf::blenfont
  PRIVATE bf::blenlib
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
)

blender_add_lib(bf_gpencil_modifiers_legacy "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.hh
add_dependencies(bf_gpencil_modifiers_legacy bf_rna)
