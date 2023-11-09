/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_image_view.hh"
#include "vk_backend.hh"
#include "vk_debug.hh"
#include "vk_device.hh"
#include "vk_memory.hh"
#include "vk_texture.hh"

namespace blender::gpu {

static VkFormat to_non_srgb_format(const VkFormat format)
{
  switch (format) {
    case VK_FORMAT_R8G8B8_SRGB:
      return VK_FORMAT_R8G8B8_UNORM;
    case VK_FORMAT_R8G8B8A8_SRGB:
      return VK_FORMAT_R8G8B8A8_UNORM;

    default:
      break;
  }
  return format;
}

VKImageView::VKImageView(VkImageViewCreateInfo &vk_image_view_info,
                         bool use_stencil,
                         IndexRange mip_range,
                         IndexRange layer_range)
{
  const VkImageAspectFlags allowed_bits = VK_IMAGE_ASPECT_COLOR_BIT |
                                          (use_stencil ? VK_IMAGE_ASPECT_STENCIL_BIT :
                                                         VK_IMAGE_ASPECT_DEPTH_BIT);

  use_stencil_ = use_stencil;
  view_type_ = vk_image_view_info.viewType;
  mip_range_ = mip_range;
  layer_range_ = layer_range;

  vk_image_view_info.subresourceRange.aspectMask = vk_image_view_info.subresourceRange.aspectMask &
                                                   allowed_bits;
  vk_image_view_info.subresourceRange.baseMipLevel = mip_range.first();
  vk_image_view_info.subresourceRange.levelCount = mip_range.size();

  auto layer_size = layer_range.size();
  vk_image_view_info.subresourceRange.baseArrayLayer = (layer_size == 0) ? 0 : layer_range.first();
  vk_image_view_info.subresourceRange.layerCount = (layer_size == 0) ? VK_REMAINING_ARRAY_LAYERS :
                                                                       layer_size;
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateImageView(
      device.device_get(), &vk_image_view_info, vk_allocation_callbacks, &vk_image_view_);
}

VKImageView::~VKImageView()
{
  VKDevice &device = VKBackend::get().device_get();
  if (vk_image_view_ != VK_NULL_HANDLE) {
    device.discard_image_view(vk_image_view_);
  }
  vk_image_view_ = VK_NULL_HANDLE;
}

bool vk_image_view_equal(std::weak_ptr<VKImageView> a, std::weak_ptr<VKImageView> b)
{
  return a.lock()->vk_handle() == b.lock()->vk_handle();
}

}  // namespace blender::gpu
