# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright 2021 Blender Foundation. All rights reserved.

set(INC
  .
  include
  ${HIPRT_INCLUDE_DIR}
)

set(INC_SYS

)

set(SRC
  src/hipew.c
  src/hiprtew.cc

  include/hipew.h
  include/hiprtew.h
)

set(LIB
)


blender_add_lib(extern_hipew "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
