# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Build meshoptimizer library.
add_subdirectory(meshoptimizer)

set(INC
  meshoptimizer/src
)

set(LIB
  meshoptimizer
)

add_library(extern_meshoptimizer SHARED "${INC}")
target_link_libraries(extern_meshoptimizer PUBLIC "${LIB}")

blender_source_group(extern_meshoptimizer "${SRC}")
