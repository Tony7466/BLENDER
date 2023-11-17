/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {
class VKTexture;
class VKImageView;
/**
 * The reason why it is necessary to generate multiple ImageViews is
 *  to improve the efficiency of the view level by the following items.
 *
 *  Required Variations
 *  -Layout (Whether to use it as an attachment or in a shader.)
 *
 *  Special Variations
 *  -Layer (Multi-layer permutations.)
 *  -Mipmap (Mipmap range.)
 *  -Viewtype (It has a close relationship with the type of the `VkImage`. CubeMap, 3D, and
 * 2D-Array are compatible.)
 *
 */
class VKImageViews {
 private:
  enum class Location : uint8_t { SHADER_BINDING, ATTCHMENT, LOCATION_ALL };
  /**
   * Types for restriction
   *
   * BASIC: required variations
   *
   * MULTI_LAYERS : layer > 1
   * MIPMAPS : mipmaps generated
   * ARRAY : there is a variation of the Viewtype
   */
  enum ImageType {
    IMAGE_TYPE_BASIC,
    IMAGE_TYPE_MULTI_LAYERS = 1,
    IMAGE_TYPE_MIPMAPS = 1 << 2,
    IMAGE_TYPE_ARRAY = 1 << 3
  };
  VkImage vk_image_ref_ = VK_NULL_HANDLE;
  ImageType image_type_;
  std::array<std::shared_ptr<VKImageView>, static_cast<size_t>(Location::LOCATION_ALL)>
      image_views_;
  VkComponentMapping vk_component_mapping_ = {VK_COMPONENT_SWIZZLE_IDENTITY,
                                              VK_COMPONENT_SWIZZLE_IDENTITY,
                                              VK_COMPONENT_SWIZZLE_IDENTITY,
                                              VK_COMPONENT_SWIZZLE_IDENTITY};

 public:
  VKImageViews() : vk_image_ref_(VK_NULL_HANDLE), image_type_(ImageType::IMAGE_TYPE_BASIC){};
  /**
   * Since there are multiple Views for one VkImage, select the type for the efficiency of the
   *cache.
   **/
  VKImageViews(VkImage vk_image, const VkImageCreateInfo &image_info);
  ~VKImageViews(){};

  std::weak_ptr<VKImageView> lookup_vk_handle(VKTexture &texture,
                                              eImageViewUsage usage,
                                              IndexRange layer_range,
                                              IndexRange mip_range,
                                              bool use_stencil,
                                              bool use_srgb,
                                              StringRefNull name);
  void swizzle_set(const char swizzle_mask[4]);
};

bool vk_image_view_equal(std::weak_ptr<VKImageView> a, std::weak_ptr<VKImageView> b);

}  // namespace blender::gpu
