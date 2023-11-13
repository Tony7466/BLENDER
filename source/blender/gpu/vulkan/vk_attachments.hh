
/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once
#include "BLI_utility_mixins.hh"

#include "vk_common.hh"
#include "vk_texture.hh"

namespace blender::gpu {

namespace vk_attachment {
const VkAttachmentDescription2 descriptions_default = {
    /*.sType*/ VK_STRUCTURE_TYPE_ATTACHMENT_DESCRIPTION_2,
    /*.pNext*/ VK_NULL_HANDLE,
    /*.flags*/ 0,
    /*.format*/ VK_FORMAT_UNDEFINED,
    /*.samples*/ VK_SAMPLE_COUNT_1_BIT,
    /*.loadOp*/ VK_ATTACHMENT_LOAD_OP_LOAD,
    /*.storeOp*/ VK_ATTACHMENT_STORE_OP_STORE,
    /*.stencilLoadOp*/ VK_ATTACHMENT_LOAD_OP_LOAD,
    /*.stencilStoreOp*/ VK_ATTACHMENT_STORE_OP_STORE};
const VkAttachmentReference2 references_default = {
    /*VkStructureType sType*/ VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2,
    /*const void *pNext*/ VK_NULL_HANDLE,
    /*uint32_t attachment*/ VK_ATTACHMENT_UNUSED,
    /*VkImageLayout layout*/ VK_IMAGE_LAYOUT_UNDEFINED,
    /*VkImageAspectFlags aspectMask*/ 0};
typedef std::array<VkAttachmentDescription2, GPU_FB_MAX_ATTACHMENT> descriptions_ty;
typedef std::array<VkAttachmentReference2, GPU_FB_MAX_ATTACHMENT - 2> references_ty;
typedef std::array<int, GPU_FB_MAX_ATTACHMENT> idx_ty;
const descriptions_ty descriptions = {descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default,
                                      descriptions_default};
const references_ty references = {references_default,
                                  references_default,
                                  references_default,
                                  references_default,
                                  references_default,
                                  references_default,
                                  references_default,
                                  references_default};
}  // namespace vk_attachment

class VKAttachments {
 private:
  std::array<vk_attachment::descriptions_ty, 2> descriptions_ = {vk_attachment::descriptions,
                                                                 vk_attachment::descriptions};
  std::array<vk_attachment::references_ty, 2> references_ = {vk_attachment::references,
                                                             vk_attachment::references};
  std::array<VkAttachmentReference2, 2> depth_references_ = {vk_attachment::references_default,
                                                             vk_attachment::references_default};
  std::array<vk_attachment::idx_ty, 2> idx_;

 public:
  void description_set(GPUTexture *texture,
                       const VkAttachmentReference2 &attachment_reference,
                       VkAttachmentDescription2 &attachment_description,
                       eRenderpassType &render_pass_enum_);

  int type_get(int view_index, int info_id) const;

  bool is_valid() const;

  friend class VKRenderPass;
  friend class VKFrameBuffer;
};

}  // namespace blender::gpu
