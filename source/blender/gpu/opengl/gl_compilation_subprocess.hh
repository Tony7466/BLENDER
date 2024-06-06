/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "GPU_compilation_subprocess.hh"

#if BLI_SUBPROCESS_SUPPORT

#  include "BLI_sys_types.h"

namespace blender::gpu {

/* The size of the memory pools shared by Blender and the compilation subprocesses. */
constexpr size_t compilation_subprocess_shared_memory_size = 1024 * 1024 * 5; /* 5mB */

struct ShaderSourceHeader {
  enum Type { COMPUTE, GRAPHICS, GRAPHICS_WITH_GEOMETRY_STAGE };
  /* The type of program being compiled. */
  Type type;
  /* When casting a shared memory pool into a ShaderSourceHeader*, this is the first char of the
   * first stage source. The stages follows the execution order (eg. vert > geom > frag). */
  char source_start;
};

struct ShaderBinaryHeader {
  /* Size of the shader binary data. */
  int32_t size;
  /* Magic number that identifies the format of this shader binary (Driver-defined).
   * This (and size) is set to 0 when the shader has failed to compile. */
  uint32_t format;
  /* When casting a shared memory pool into a ShaderBinaryHeader*, this is the first byte of the
   * shader binary data. */
  uint8_t data_start;
};

}  // namespace blender::gpu

#endif
