/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "gpu_framebuffer_private.hh"

#include "vk_common.hh"
#include "vk_image_view.hh"
#include "vk_renderpass.hh"

#include <optional>

namespace blender::gpu {
namespace vk_framebuffer {
const VkFramebufferCreateInfo create_info_default = {
    /*VkStructureType sType;*/ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO,
    /*const void *pNext;*/ VK_NULL_HANDLE,
    /*VkFramebufferCreateFlags flags;*/ 0,
    /*VkRenderPass renderPass;*/ VK_NULL_HANDLE,
    /*uint32_t attachmentCount;*/ 0,
    /*const VkImageView *pAttachments;*/ nullptr,
    /*uint32_t width;*/ 0,
    /*uint32_t height;*/ 0,
    /*uint32_t layers;*/ 0};
}

class VKFrameBuffer : public FrameBuffer {
 private:
  /* Vulkan object handle. */
  VkFramebuffer vk_framebuffer_ = VK_NULL_HANDLE;

  /* Vulkan device who created the handle. */
  VkDevice vk_device_ = VK_NULL_HANDLE;
  /**
   * One VkRenderPass corresponds to one VKFramebuffer.
   * In the current design, clear_pass is not done.
   */
  std::optional<VKRenderPass> renderpass_;
  /* Number of layers if the attachments are layered textures. */
  int depth_ = 1;

  std::array<std::weak_ptr<VKImageView>, GPU_FB_MAX_ATTACHMENT> image_views_;

  VkFramebufferCreateInfo vk_framebuffer_create_info_ = vk_framebuffer::create_info_default;

  uint8_t dirty_attachments_8_ = 0;
  uint8_t default_attachments_ = 0;
  /** Is the first attachment an SRGB texture. */
  bool srgb_;
  bool enabled_srgb_;

 public:
  /**
   * Create a conventional framebuffer to attach texture to.
   **/
  VKFrameBuffer(const char *name);

  ~VKFrameBuffer();

  void bind(bool enabled_srgb) override;
  bool check(char err_out[256]) override;
  void clear(eGPUFrameBufferBits buffers,
             const float clear_color[4],
             float clear_depth,
             uint clear_stencil) override;
  void clear_multi(const float (*clear_color)[4]) override;
  void clear_attachment(GPUAttachmentType type,
                        eGPUDataFormat data_format,
                        const void *clear_value) override;

  void attachment_set_loadstore_op(GPUAttachmentType type, GPULoadStore /*ls*/) override;
  void config(const GPUAttachment *config, int config_len) override;
  void subpass_transition(const GPUAttachmentState depth_attachment_state,
                          Span<GPUAttachmentState> color_attachment_states) override;

  void read(eGPUFrameBufferBits planes,
            eGPUDataFormat format,
            const int area[4],
            int channel_len,
            int slot,
            void *r_data) override;

  void blit_to(eGPUFrameBufferBits planes,
               int src_slot,
               FrameBuffer *dst,
               int dst_slot,
               int dst_offset_x,
               int dst_offset_y) override;

  void attachment_set(GPUAttachmentType type,
                      const GPUAttachment &new_attachment,
                      bool config = true) override;

  void cache_init();

  bool is_valid() const
  {
    return vk_framebuffer_ != VK_NULL_HANDLE;
  }

  VkRenderPass vk_render_pass_get();
  VkFramebuffer vk_framebuffer_get()
  {
    if (vk_framebuffer_ == VK_NULL_HANDLE) {
      ensure();
    };
    return vk_framebuffer_;
  }
  Array<VkViewport, 16> vk_viewports_get() const;
  Array<VkRect2D, 16> vk_render_areas_get() const;

  void depth_attachment_layout_ensure(VKContext &context, VkImageLayout requested_layout);
  void color_attachment_layout_ensure(VKContext &context,
                                      int color_attachment,
                                      VkImageLayout requested_layout);
  /**
   * Ensure that the size of the frame-buffer matches the first attachment resolution.
   *
   * Frame buffers attachments are updated when actually used as the image layout has to be
   * correct. After binding frame-buffers the layout of images can still be modified.
   *
   * But for correct behavior of blit/clear operation the size of the frame-buffer should be
   * set, when activating the frame buffer.
   */
  void update_size();

  void update_srgb();

  /**
   * Return the number of color attachments of this frame buffer, including unused color
   * attachments.
   *
   * Frame-buffers can have unused attachments. When higher attachment slots are being used, unused
   * lower attachment slots will be counted as they are required resources in render-passes.
   */
  int color_attachments_resource_size() const;

  void renderpass_ensure();

 private:
  void update_attachments();
  void create();
  void free();
  void ensure();
  bool image_view_ensure(GPUTexture *tex, int mip, IndexRange &layer_range, int attachment_index);
  /* Clearing attachments */
  void build_clear_attachments_depth_stencil(eGPUFrameBufferBits buffers,
                                             float clear_depth,
                                             uint32_t clear_stencil,
                                             Vector<VkClearAttachment> &r_attachments) const;
  void build_clear_attachments_color(const float (*clear_colors)[4],
                                     const bool multi_clear_colors,
                                     Vector<VkClearAttachment> &r_attachments) const;
  void clear(const Vector<VkClearAttachment> &attachments) const;
};

static inline VKFrameBuffer *unwrap(FrameBuffer *framebuffer)
{
  return static_cast<VKFrameBuffer *>(framebuffer);
}

}  // namespace blender::gpu
