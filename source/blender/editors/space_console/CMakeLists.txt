# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenkernel
  ../../blenloader
  ../../gpu
  ../../makesrna
  ../../windowmanager

)

set(INC_SYS
)

set(SRC
  console_draw.cc
  console_ops.cc
  space_console.cc

  console_intern.hh
)

set(LIB
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf::dna
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_space_info
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)

if(WITH_PYTHON)
  add_definitions(-DWITH_PYTHON)
endif()


blender_add_lib(bf_editor_space_console "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
