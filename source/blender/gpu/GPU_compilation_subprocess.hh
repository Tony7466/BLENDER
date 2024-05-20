/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdint>

namespace blender::gpu {

struct ShaderBinary {
  static const size_t max_data_size = 1024 * 1024 * 5; /* 5mB */
  int32_t size;
  uint32_t format;
  uint8_t data_start;
};

}  // namespace blender::gpu

void GPU_compilation_subprocess_run(const char *subprocess_name);
