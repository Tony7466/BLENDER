/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_renderpass.hh"

namespace blender::gpu {

enum class SubpassTransitionPattern : uint8_t {
  EXTERNAL_TO_COLOR = 0,
  EXTERNAL_SHADER_READ_WRITE_TO_COLOR,
  EXTERNAL_SHADER_READ_WRITE_TO_DEPTH,
  COLOR_TO_INPUT,
  INPUT_TO_EXTERNAL_COLOR,
  INPUT_TO_EXTERNAL_SHADER_READ_WRITE,
  COLOR_TO_EXTERNAL,
  /* If the bits can be reversed, do so.*/
  DEPTH_TO_EXTERNAL_SHADER_READ_WRITE = 0b1100,
  COLOR_TO_EXTERNAL_SHADER_READ_WRITE = 0b1101,
  EXTERNAL_TO_DEPTH = 0b1111,

  SUBPASS_TRANSITION_PATTERN_ALL
};

/* Multiple subpasses dynamically generate chains from dependency types. */
struct VKSubpassDependency : public VkSubpassDependency2 {
 public:
  VKSubpassDependency()
  {
    sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2;
    pNext = VK_NULL_HANDLE;
    /* dependencyFlags is used in MultiView etc. */
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    viewOffset = 0;
  }
  VkSubpassDependency2 &color_to_input(uint32_t src_pass, uint32_t dst_pass)
  {
    srcSubpass = src_pass;
    dstSubpass = dst_pass;
    srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dstAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &input_to_external(uint32_t src_pass, VkImageLayout dst_layout)
  {
    BLI_assert(ELEM(dst_layout,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_GENERAL));
    srcSubpass = src_pass;
    dstSubpass = VK_SUBPASS_EXTERNAL;
    srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &color_to_external(uint32_t src_pass, VkImageLayout dst_layout)
  {
    BLI_assert(ELEM(dst_layout,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_IMAGE_LAYOUT_GENERAL));
    srcSubpass = src_pass;
    dstSubpass = VK_SUBPASS_EXTERNAL;
    srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dstAccessMask = to_vk_layout_to_access_flag(dst_layout);
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);

    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &external_to_color(uint32_t dst_pass, VkImageLayout src_layout)
  {
    BLI_assert(ELEM(src_layout,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_IMAGE_LAYOUT_GENERAL,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL));
    srcSubpass = VK_SUBPASS_EXTERNAL;
    dstSubpass = dst_pass;
    srcAccessMask = to_vk_layout_to_access_flag(src_layout);
    dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &external_to_depth(uint32_t dst_pass, VkImageLayout src_layout)
  {
    BLI_assert(ELEM(src_layout,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL));
    srcSubpass = VK_SUBPASS_EXTERNAL;
    dstSubpass = dst_pass;
    srcAccessMask = to_vk_layout_to_access_flag(src_layout);
    dstAccessMask = to_vk_layout_to_access_flag(VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &depth_to_external(uint32_t src_pass, VkImageLayout dst_layout)
  {
    BLI_assert(ELEM(dst_layout, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));

    srcSubpass = src_pass;
    dstSubpass = VK_SUBPASS_EXTERNAL;
    srcAccessMask = to_vk_layout_to_access_flag(VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    dstAccessMask = to_vk_layout_to_access_flag(dst_layout);
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);

    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &color_to_color(uint32_t pass)
  {
    srcSubpass = pass;
    dstSubpass = pass;
    srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
    dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &depth_to_depth(uint32_t src_pass, uint32_t dst_pass)
  {
    srcSubpass = src_pass;
    dstSubpass = dst_pass;
    srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = to_vk_access_to_stage_flag(dstAccessMask);
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &write_to_read(uint32_t src_pass, uint32_t dst_pass)
  {
    srcSubpass = src_pass;
    dstSubpass = dst_pass;
    srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
    dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    srcStageMask = to_vk_access_to_stage_flag(srcAccessMask);
    dstStageMask = VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }

  VkSubpassDependency2 &write_to_read2(uint32_t src_pass, uint32_t dst_pass)
  {
    srcSubpass = src_pass;
    dstSubpass = dst_pass;
    srcAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    srcStageMask = VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dstStageMask = VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
    return *reinterpret_cast<VkSubpassDependency2 *>(this);
  }
};
}  // namespace blender::gpu
