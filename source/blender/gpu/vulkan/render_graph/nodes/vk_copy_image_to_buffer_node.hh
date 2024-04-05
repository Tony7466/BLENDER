/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKCopyImageToBufferNode:NonCopyable {
  struct Data {
    VkImage src_image;
    VkBuffer dst_buffer;
    VkBufferImageCopy region;
  };
};
}  // namespace blender::gpu::render_graph
