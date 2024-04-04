/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKFillBufferNode:NonCopyable {
  struct Data {
    VkBuffer vk_buffer;
    VkDeviceSize size;
    uint32_t data;
  };
};
}  // namespace blender::gpu::render_graph
