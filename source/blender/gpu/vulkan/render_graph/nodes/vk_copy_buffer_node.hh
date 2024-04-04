/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKCopyBufferNode {
  struct Data {
    VkBuffer src_buffer;
    VkBuffer dst_buffer;
    VkBufferCopy region;
  };
};
}  // namespace blender::gpu::render_graph
