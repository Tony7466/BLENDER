/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKCopyImageNode {
  struct Data {
    VkImage src_image;
    VkImage dst_image;
    VkImageCopy region;
  };
};
}  // namespace blender::gpu::render_graph
