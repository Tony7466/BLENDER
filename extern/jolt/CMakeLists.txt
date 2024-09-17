# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Build the Jolt library first.
add_subdirectory(src)

# Create a wrapper to avoid leaking build settings from Jolt to Blender.
add_library(extern_jolt INTERFACE)
target_link_libraries(extern_jolt INTERFACE Jolt)