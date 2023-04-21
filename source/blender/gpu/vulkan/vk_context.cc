/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

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

#include "GHOST_C-api.h"

namespace blender::gpu {

VKContext::VKContext(void *ghost_window, void *ghost_context)
{
  VK_ALLOCATION_CALLBACKS;
  ghost_window_ = ghost_window;
  if (ghost_window) {
    ghost_context = GHOST_GetDrawingContext((GHOST_WindowHandle)ghost_window);
  }
  ghost_context_ = ghost_context;

  GHOST_GetVulkanHandles((GHOST_ContextHandle)ghost_context,
                         &vk_instance_,
                         &vk_physical_device_,
                         &vk_device_,
                         &vk_queue_family_,
                         &vk_queue_);
  debug::init_callbacks(this, vkGetInstanceProcAddr);
  init_physical_device_limits();

  debug::object_label(this, vk_device_, "LogicalDevice");
  debug::object_label(this, vk_queue_, "GenericQueue");

  /* Initialize the memory allocator. */
  VmaAllocatorCreateInfo info = {};
  /* Should use same vulkan version as GHOST (1.2), but set to 1.0 as 1.2 requires
   * correct extensions and functions to be found by VMA, which isn't working as expected and
   * requires more research. To continue development we lower the API to version 1.0. */
  info.vulkanApiVersion = VK_API_VERSION_1_0;
  info.physicalDevice = vk_physical_device_;
  info.device = vk_device_;
  info.instance = vk_instance_;
  info.pAllocationCallbacks = vk_allocation_callbacks;
  vmaCreateAllocator(&info, &mem_allocator_);
  descriptor_pools_.init(vk_device_);

  state_manager = new VKStateManager(*this);
  imm = new VKImmediate();

  VKBackend::capabilities_init(*this);

  /* For off-screen contexts. Default frame-buffer is empty. */
  VKFrameBuffer *framebuffer = new VKFrameBuffer("back_left");
  back_left = framebuffer;
  active_fb = framebuffer;
}

VKContext::~VKContext()
{
  /* IMM owns a buffer that should be freed before the memory allocator is freed. */
  /* TODO: Introduce a buffer pool for temporary buffers. */
  delete imm;
  imm = nullptr;
  vmaDestroyAllocator(mem_allocator_);
  debug::destroy_callbacks(this);
}

void VKContext::init_physical_device_limits()
{
  BLI_assert(vk_physical_device_ != VK_NULL_HANDLE);
  VkPhysicalDeviceProperties properties = {};
  vkGetPhysicalDeviceProperties(vk_physical_device_, &properties);
  vk_physical_device_limits_ = properties.limits;
}

void VKContext::activate()
{
  /* Make sure no other context is already bound to this thread. */
  BLI_assert(is_active_ == false);

  is_active_ = true;

  if (ghost_window_) {
    VkImage image; /* TODO will be used for reading later... */
    VkFramebuffer vk_framebuffer;
    VkRenderPass render_pass;
    VkExtent2D extent;
    uint32_t fb_id;

    GHOST_GetVulkanBackbuffer(
        (GHOST_WindowHandle)ghost_window_, &image, &vk_framebuffer, &render_pass, &extent, &fb_id);

    /* Recreate the gpu::VKFrameBuffer wrapper after every swap. */
    if (has_active_framebuffer()) {
      deactivate_framebuffer();
    }
    delete back_left;

    VKFrameBuffer *framebuffer = new VKFrameBuffer(
        "back_left", vk_framebuffer, render_pass, extent);
    back_left = framebuffer;
    framebuffer->bind(false);
    active_fb = back_left;
  }

  immActivate();
}

void VKContext::deactivate()
{
  immDeactivate();
  is_active_ = false;
}

void VKContext::begin_frame()
{
  printf("[%s]\n", __func__);
  VkCommandBuffer command_buffer = VK_NULL_HANDLE;
  GHOST_GetVulkanCommandBuffer(static_cast<GHOST_ContextHandle>(ghost_context_), &command_buffer);
  command_buffer_.init(vk_device_, vk_queue_, command_buffer);
  command_buffer_.begin_recording();

  descriptor_pools_.reset();
}

void VKContext::end_frame()
{
  command_buffer_.end_recording();
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

const VKStateManager &VKContext::state_manager_get() const
{
  return *static_cast<const VKStateManager *>(state_manager);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Framebuffer
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
/** \name Graphics pipeline
 * \{ */

void VKContext::bind_graphics_pipeline(const GPUPrimType prim_type,
                                       const VKVertexAttributeObject &vertex_attribute_object)
{
  VKShader *shader = unwrap(this->shader);
  BLI_assert(shader);
  shader->update_graphics_pipeline(*this, prim_type, vertex_attribute_object);
  command_buffer_get().bind(shader->pipeline_get(), VK_PIPELINE_BIND_POINT_GRAPHICS);

  VKPipeline &pipeline = shader->pipeline_get();
  VKDescriptorSetTracker &descriptor_set = pipeline.descriptor_set_get();
  VKPushConstants &push_constants = pipeline.push_constants_get();

  /* TODO move into pipeline. See VKBackend::compute_dispatch. */
  push_constants.update(*this);
  if (descriptor_set.has_layout()) {
    descriptor_set.update(*this);
    command_buffer_.bind(*descriptor_set.active_descriptor_set(),
                         shader->vk_pipeline_layout_get(),
                         VK_PIPELINE_BIND_POINT_GRAPHICS);
  }
}

/** \} */

}  // namespace blender::gpu
