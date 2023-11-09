/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

#include "BLI_string_ref.hh"
#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKTexture;

class VKImageView : NonCopyable {
 private:
  VkImageView vk_image_view_ = VK_NULL_HANDLE;
  bool use_stencil_;
  VkImageViewType view_type_;
  IndexRange mip_range_;
  IndexRange layer_range_;

 public:
  VKImageView(VkImageViewCreateInfo &vk_image_view_info_,
              bool use_stencil,
              IndexRange mip_range,
              IndexRange layer_range);

  VKImageView(VKImageView &&other);
  ~VKImageView();

  VkImageView vk_handle() const
  {
    BLI_assert(vk_image_view_ != VK_NULL_HANDLE);
    return vk_image_view_;
  }

  bool check_eq(bool use_stencil) const
  {
    return use_stencil == use_stencil_;
  }
  bool check_eq(VkImageViewType view_type) const
  {
    return view_type == view_type_;
  }
  bool check_eq(IndexRange range, bool layer) const
  {
    if (layer) {
      return layer_range_ == range;
    }
    return mip_range_ == range;
  }
};

}  // namespace blender::gpu
