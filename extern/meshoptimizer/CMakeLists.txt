# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Build meshoptimizer library.
add_subdirectory(meshoptimizer)

# Build meshoptimizer-Blender bridging module.
set(SRC
  src/common.h
  src/decoder.cpp
  src/decoder.h
)

set(INC
  meshoptimizer/src
)

set(LIB
  meshoptimizer
)

add_library(extern_meshoptimizer SHARED "${SRC}")
target_include_directories(extern_meshoptimizer PUBLIC "${INC}")
target_link_libraries(extern_meshoptimizer PUBLIC "${LIB}")

blender_source_group(extern_meshoptimizer "${SRC}")
