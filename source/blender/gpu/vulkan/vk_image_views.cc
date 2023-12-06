/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_image_views.hh"
#include "vk_image_view.hh"
#include "vk_texture.hh"

#include "BLI_assert.h"

namespace blender::gpu {

VKImageViews::VKImageViews(VkImage vk_image, const VkImageCreateInfo &image_info)
{
  image_type_ = IMAGE_TYPE_BASIC;

  if (image_info.mipLevels > 1) {
    image_type_ = static_cast<ImageType>(image_type_ | IMAGE_TYPE_MIPMAPS);
  }

  if (image_info.arrayLayers > 1) {
    image_type_ = static_cast<ImageType>(image_type_ | IMAGE_TYPE_MULTI_LAYERS);
  }

  if (image_info.flags & VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT) {
    image_type_ = static_cast<ImageType>(image_type_ | IMAGE_TYPE_ARRAY);
  }

  vk_image_ref_ = vk_image;
}

int VKImageViews::location_get(eImageViewUsage usage)
{
  int location = -1;
  switch (usage) {
    case eImageViewUsage::ShaderBinding:
      location = static_cast<int>(Location::SHADER_BINDING);
      break;
    case eImageViewUsage::Attachment:
      location = static_cast<int>(Location::ATTCHMENT);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }
  return location;
}

std::weak_ptr<VKImageView> VKImageViews::lookup_vk_handle(VKTexture &texture,
                                                          eImageViewUsage usage,
                                                          IndexRange layer_range,
                                                          IndexRange mip_range,
                                                          bool use_stencil,
                                                          bool use_srgb,
                                                          StringRefNull name)
{
  BLI_assert(texture.vk_image_handle() == vk_image_ref_);
  int location = location_get(usage);
  const eGPUTextureFormat format = texture.device_format_get();

  VkImageViewCreateInfo vk_image_view_info_ = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
                                               VK_NULL_HANDLE};

  bool enabled_srgb = (texture.format_flag_get() & GPU_FORMAT_SRGB && use_srgb);

  if (enabled_srgb) {
    vk_image_view_info_.format = to_vk_format(format);
  }
  else {
    vk_image_view_info_.format = to_non_srgb_format(to_vk_format(format));
  }
  vk_image_view_info_.components = vk_component_mapping_;
  vk_image_view_info_.subresourceRange.aspectMask = to_vk_image_aspect_flag_bits(format);

  vk_image_view_info_.viewType = to_vk_image_view_type(texture.type_get(), usage);

  std::shared_ptr<VKImageView> image_view = image_views_[location];
  bool need_create = false;

  if (!image_view.get()) {
    need_create = true;
  }
  else {
    need_create |= !image_view->check_srgb(vk_image_view_info_.format);
    need_create |= !image_view->check_eq(use_stencil);
    if (image_type_ & IMAGE_TYPE_ARRAY) {
      need_create |= !image_view->check_eq(vk_image_view_info_.viewType);
    }
    if (image_type_ & IMAGE_TYPE_MULTI_LAYERS) {
      need_create |= !image_view->check_eq(layer_range, true);
    }
    if (image_type_ & IMAGE_TYPE_MIPMAPS) {
      need_create |= !image_view->check_eq(mip_range, false);
    }
  }

  if (need_create) {
    vk_image_view_info_.image = vk_image_ref_;
    image_views_[location] = std::shared_ptr<VKImageView>(
        new VKImageView(vk_image_view_info_, use_stencil, mip_range, layer_range));
    debug::object_label(image_views_[location]->vk_handle(), name.c_str());
  }

  return image_views_[location];
}

void VKImageViews::swizzle_set(const char swizzle_mask[4])
{
  vk_component_mapping_.r = to_vk_component_swizzle(swizzle_mask[0]);
  vk_component_mapping_.g = to_vk_component_swizzle(swizzle_mask[1]);
  vk_component_mapping_.b = to_vk_component_swizzle(swizzle_mask[2]);
  vk_component_mapping_.a = to_vk_component_swizzle(swizzle_mask[3]);
};

}  // namespace blender::gpu
