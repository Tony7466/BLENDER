/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKClearColorImageNode:NonCopyable {
  struct Data {
    VkImage vk_image;
    VkClearColorValue vk_clear_color_value;
    VkImageSubresourceRange vk_image_subresource_range;
  };
};
}  // namespace blender::gpu::render_graph
