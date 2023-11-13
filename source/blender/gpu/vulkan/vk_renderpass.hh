/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once
#include "vk_attachments.hh"
#include "vk_common.hh"

#include <memory>

namespace blender::gpu {
/*
 * Implementing Image Transitions in the `VkRenderPass`.
 * Of course, there are various transitions that can be considered, but with the following two
 * transition types, the images used for most renders can be used without being barrierd in the
 * middle.
 *
 * type0:  `VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL` to `VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL`
 * type1:  `VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL` to `VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL`
 *
 */
namespace vk_renderpass {
const VkRenderPassCreateInfo2 create_info_default = {VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO_2,
                                                     VK_NULL_HANDLE,
                                                     0,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr};
}
namespace vk_subpass {
const VkSubpassDescription2 descriptions_default = {
    VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_2, VK_NULL_HANDLE, 0, VK_PIPELINE_BIND_POINT_GRAPHICS};
}

/**
 * Pipeline can be a compute pipeline or a graphic pipeline.
 *
 * Compute pipelines can be constructed early on, but graphics
 * pipelines depends on the actual GPU state/context.
 *
 * - TODO: we should sanitize the interface. There we can also
 *   use late construction for compute pipelines.
 */

class VKRenderPass {
 private:
  bool dirty_ = false;
  VkRenderPass vk_render_pass_ = VK_NULL_HANDLE;
  eRenderpassType render_pass_enum_ = eRenderpassType::Any;
  VkRenderPassCreateInfo2 vk_create_info_[2] = {vk_renderpass::create_info_default,
                                                vk_renderpass::create_info_default};
  uint8_t info_id_ = 0b00;
  const uint8_t info_id_counter() const
  {
    return (info_id_ + 1) % 2;
  }

  /** There is a separate classification issue with regard to multi-layered rendering. **/
  int multiview_layers_ = 1;
  std::array<VkSubpassDescription2, 2> subpass_ = {vk_subpass::descriptions_default,
                                                   vk_subpass::descriptions_default};

  VKAttachments attachments_;

 public:
  VKRenderPass(){};
  ~VKRenderPass()
  {
    free();
  };
  void create();
  void free();
  bool ensure();
  void cache_init();
  void dependency_set(bool use_depth);
  void multiview_set();
  friend class VKFrameBuffer;
};

}  // namespace blender::gpu
