# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
)

set(INC_SYS
)

set(SRC
  # in order for cmake PCH REUSE_FROM to work with projects
  # that use a mix of C and C++, the PCH library needs to
  # have both languages in there, otherwise you get an
  # "Unable to resolve full path of PCH-header '<somepath>' assigned to target <target>, although its path is supposed to be known!"
  # error.
  pch.c
  pch.cc
)

set(LIB
)

blender_add_lib(bf_pch "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(COMMAND target_precompile_headers)
  target_precompile_headers(bf_pch PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/pch.h")
endif()
