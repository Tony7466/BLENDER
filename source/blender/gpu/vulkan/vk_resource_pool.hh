/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {
class VKResourcePool {
 private:
  Vector<std::pair<VkImage, VmaAllocation>> discarded_images_;
  Vector<std::pair<VkBuffer, VmaAllocation>> discarded_buffers_;
  Vector<VkImageView> discarded_image_views_;

 public:
  void discard_image(VkImage vk_image, VmaAllocation vma_allocation);
  void discard_image_view(VkImageView vk_image_view);
  void discard_buffer(VkBuffer vk_buffer, VmaAllocation vma_allocation);
  void destroy_discarded_resources();
};
}  // namespace blender::gpu
