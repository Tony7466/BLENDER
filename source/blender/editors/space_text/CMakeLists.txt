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

)

set(INC_SYS
)

set(SRC
  space_text.cc
  text_autocomplete.cc
  text_draw.cc
  text_format.cc
  text_format_glsl.cc
  text_format_osl.cc
  text_format_pov.cc
  text_format_pov_ini.cc
  text_format_py.cc
  text_header.cc
  text_ops.cc
  text_undo.cc

  text_format.hh
  text_intern.hh
)

set(LIB
  PRIVATE bf::blenfont
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf_blenloader
  PRIVATE bf_blentranslation
  PRIVATE bf::dna
  PRIVATE bf_editor_curve
  PRIVATE bf_editor_interface
  PRIVATE bf_editor_screen
  PRIVATE bf_editor_undo
  PRIVATE bf_gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf_python
  PRIVATE bf_rna
  PRIVATE bf_windowmanager
)


if(WITH_PYTHON)
  list(APPEND INC
    ../../python
  )
  add_definitions(-DWITH_PYTHON)
endif()


blender_add_lib(bf_editor_space_text "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
