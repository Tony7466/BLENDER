/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */
#include "vk_context.hh"
#include "vk_debug.hh"

#include "vk_backend.hh"
#include "vk_framebuffer.hh"
#include "vk_immediate.hh"
#include "vk_memory.hh"
#include "vk_shader.hh"
#include "vk_state_manager.hh"
#include "vk_texture.hh"

#include "GHOST_C-api.h"

namespace blender::gpu {

VKContext::VKContext(void *ghost_window, void *ghost_context)
{
  ghost_window_ = ghost_window;
  ghost_context_ = ghost_context;

  state_manager = new VKStateManager();
  imm = new VKImmediate();

  /* For off-screen contexts. Default frame-buffer is empty. */
  VKFrameBuffer *framebuffer = new VKFrameBuffer("back_left");
  back_left = framebuffer;
  active_fb = framebuffer;
}

VKContext::~VKContext()
{
  if (surface_texture_) {
    GPU_texture_free(surface_texture_);
    surface_texture_ = nullptr;
  }
  VKBackend::get().device_.context_unregister(*this);

  delete imm;
  imm = nullptr;
}

void VKContext::sync_backbuffer()
{
  if (ghost_context_) {
    /* TODO: Only needed when command buffer isn't initialized. Or when we create our own command
     * buffer.*/
    /*
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    GHOST_GetVulkanCommandBuffer(static_cast<GHOST_ContextHandle>(ghost_context_),
                                 &command_buffer);*/
    VKDevice &device = VKBackend::get().device_;
    if (!command_buffer_.is_initialized()) {
      command_buffer_.init(device);
      command_buffer_.begin_recording();
      device.init_dummy_buffer(*this);
    }
    device.descriptor_pools_get().reset();
  }

  if (ghost_window_) {
    VkSurfaceFormatKHR vk_surface_format;
    VkExtent2D extent;

    GHOST_GetVulkanBackbufferFormat(
        (GHOST_WindowHandle)ghost_window_, &vk_surface_format, &extent);

    const bool reset_framebuffer = vk_surface_format_.format != vk_surface_format.format ||
                                   vk_extent_.width != extent.width ||
                                   vk_extent_.height != extent.height;
    if (reset_framebuffer) {
      if (has_active_framebuffer()) {
        deactivate_framebuffer();
      }
      if (surface_texture_) {
        GPU_texture_free(surface_texture_);
        surface_texture_ = nullptr;
      }
      surface_texture_ = GPU_texture_create_2d("back-left",
                                               extent.width,
                                               extent.height,
                                               1,
                                               to_gpu_format(vk_surface_format.format),
                                               GPU_TEXTURE_USAGE_ATTACHMENT,
                                               nullptr);

      back_left->attachment_set(GPU_FB_COLOR_ATTACHMENT0,
                                GPU_ATTACHMENT_TEXTURE(surface_texture_));

      back_left->bind(false);

      vk_surface_format_ = vk_surface_format;
      vk_extent_ = extent;
    }
  }
}

void VKContext::activate()
{
  /* Make sure no other context is already bound to this thread. */
  BLI_assert(is_active_ == false);

  is_active_ = true;

  sync_backbuffer();

  immActivate();
}

void VKContext::deactivate()
{
  immDeactivate();
  is_active_ = false;
}

void VKContext::begin_frame()
{
  // sync_backbuffer();
}

void VKContext::end_frame()
{
  // command_buffer_.end_recording();
}

void VKContext::flush()
{
  command_buffer_.submit();
}

void VKContext::finish()
{
  command_buffer_.submit();
}

void VKContext::memory_statistics_get(int * /*total_mem*/, int * /*free_mem*/) {}

/* -------------------------------------------------------------------- */
/** \name State manager
 * \{ */

VKStateManager &VKContext::state_manager_get() const
{
  return *static_cast<VKStateManager *>(state_manager);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Frame-buffer
 * \{ */

void VKContext::activate_framebuffer(VKFrameBuffer &framebuffer)
{
  if (has_active_framebuffer()) {
    deactivate_framebuffer();
  }

  BLI_assert(active_fb == nullptr);
  active_fb = &framebuffer;
  command_buffer_.begin_render_pass(framebuffer);
}

VKFrameBuffer *VKContext::active_framebuffer_get() const
{
  return unwrap(active_fb);
}

bool VKContext::has_active_framebuffer() const
{
  return active_framebuffer_get() != nullptr;
}

void VKContext::deactivate_framebuffer()
{
  VKFrameBuffer *framebuffer = active_framebuffer_get();
  BLI_assert(framebuffer != nullptr);
  if (framebuffer->is_valid()) {
    command_buffer_.end_render_pass(*framebuffer);
  }
  active_fb = nullptr;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Compute pipeline
 * \{ */

void VKContext::bind_compute_pipeline()
{
  VKShader *shader = unwrap(this->shader);
  BLI_assert(shader);
  VKPipeline &pipeline = shader->pipeline_get();
  pipeline.update_and_bind(
      *this, shader->vk_pipeline_layout_get(), VK_PIPELINE_BIND_POINT_COMPUTE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Graphics pipeline
 * \{ */

void VKContext::bind_graphics_pipeline(const GPUPrimType prim_type,
                                       const VKVertexAttributeObject &vertex_attribute_object)
{
  VKShader *shader = unwrap(this->shader);
  BLI_assert(shader);
  shader->update_graphics_pipeline(*this, prim_type, vertex_attribute_object);

  VKPipeline &pipeline = shader->pipeline_get();
  pipeline.update_and_bind(
      *this, shader->vk_pipeline_layout_get(), VK_PIPELINE_BIND_POINT_GRAPHICS);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Graphics pipeline
 * \{ */

void VKContext::swap_buffers_pre_callback()
{
  VKContext *context = VKContext::get();
  BLI_assert(context);
  context->swap_buffers_pre_handler();
}

void VKContext::swap_buffers_post_callback()
{
  VKContext *context = VKContext::get();
  BLI_assert(context);
  context->swap_buffers_post_handler();
}

void VKContext::swap_buffers_pre_handler()
{
  VKFrameBuffer &framebuffer = *unwrap(back_left);

  /* Get swapchain image. */
  VkImage vk_image = VK_NULL_HANDLE;
  VkSurfaceFormatKHR vk_surface_format = {};
  VkFramebuffer vk_framebuffer = VK_NULL_HANDLE;
  VkRenderPass vk_render_pass = VK_NULL_HANDLE;
  VkExtent2D vk_extent = {};
  uint32_t framebuffer_id = 0;
  GHOST_GetVulkanBackbuffer(static_cast<GHOST_WindowHandle>(ghost_window_),
                            &vk_image,
                            &vk_surface_format,
                            &vk_framebuffer,
                            &vk_render_pass,
                            &vk_extent,
                            &framebuffer_id);

  VKTexture wrapper("display_texture");
  wrapper.init(vk_image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, to_gpu_format(vk_surface_format.format));
  wrapper.layout_ensure(*this, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  VKTexture *color_attachment = unwrap(unwrap(framebuffer.color_tex(0)));
  color_attachment->layout_ensure(*this, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

  VkImageBlit image_blit = {};
  image_blit.srcOffsets[0] = {0, int32_t(vk_extent.height) - 1, 0};
  image_blit.srcOffsets[1] = {int32_t(vk_extent.width), 0, 1};
  image_blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_blit.srcSubresource.mipLevel = 0;
  image_blit.srcSubresource.baseArrayLayer = 0;
  image_blit.srcSubresource.layerCount = 1;

  image_blit.dstOffsets[0] = {0, 0, 0};
  image_blit.dstOffsets[1] = {int32_t(vk_extent.width), int32_t(vk_extent.height), 1};
  image_blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_blit.dstSubresource.mipLevel = 0;
  image_blit.dstSubresource.baseArrayLayer = 0;
  image_blit.dstSubresource.layerCount = 1;

  command_buffer_.blit(wrapper,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       *color_attachment,
                       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       Span<VkImageBlit>(&image_blit, 1));
  wrapper.layout_ensure(*this, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
  command_buffer_.submit();
}

void VKContext::swap_buffers_post_handler()
{
  sync_backbuffer();
}

/** \} */

}  // namespace blender::gpu
