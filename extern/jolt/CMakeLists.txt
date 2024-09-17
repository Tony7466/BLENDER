# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Build the Jolt library first.
add_subdirectory(src)

# Create a wrapper to avoid leaking build settings from Jolt to Blender.
set(SRC
    stub.cc
)
add_library(extern_jolt ${SRC})
target_link_libraries(extern_jolt PRIVATE Jolt)