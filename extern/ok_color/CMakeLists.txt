# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright 2023 Blender Foundation

set(INC
  .
)

set(INC_SYS
)

set(SRC
  ok_color.c

  ok_color.h
)

set(LIB
)

blender_add_lib(extern_ok_color "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
