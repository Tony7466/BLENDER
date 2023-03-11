/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */


#include "vk_context.hh"
#include "vk_debug.hh"

#include "vk_backend.hh"
#include "vk_framebuffer.hh"
#include "vk_memory.hh"
#include "vk_state_manager.hh"

#include "GHOST_C-api.h"

#ifdef VK_NO_PROTOTYPES
#  if defined(__unix__) || defined(__APPLE__)
#    include <sys/time.h>
#    include <unistd.h>
#    define GET_FUNC_ADDRESS dlsym
#  endif
#  if defined(_MSC_VER)
#    include <Windows.h>

#    include <VersionHelpers.h> /* This needs to be included after Windows.h. */
#    include <io.h>
#    if !defined(ENABLE_VIRTUAL_TERMINAL_PROCESSING)
#      define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#    endif

#    define GET_FUNC_ADDRESS (void *)GetProcAddress
#  endif
#endif



namespace blender::gpu {

static VKFunctionsLoader loader;

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


  /*Load instance extended functions.*/
  {
    loader.vulkan_dynamic_load();
    loader.vulkan_dynamic_load_instance(vk_instance_);
  }

  if (vk_device_ == VK_NULL_HANDLE) {
    GHOST_GetVulkanLogicalDevice(
        (GHOST_ContextHandle)ghost_context, &vk_device_, &vk_queue_family_, &vk_queue_);
  }

  /*Load device extended functions.*/
  {
    loader.vulkan_dynamic_load_device(vk_device_);
  }

  debug::object_vk_label(vk_device_, vk_device_, std::string("LogicalDevice"));
  debug::object_vk_label(vk_device_, vk_queue_, std::string("GraphicsQueue"));

  init_physical_device_limits();

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

  state_manager = new VKStateManager();

  VKBackend::capabilities_init(*this);

  /* For off-screen contexts. Default frame-buffer is empty. */
  active_fb = back_left = new VKFrameBuffer("back_left");
}

VKContext::~VKContext()
{
  vmaDestroyAllocator(mem_allocator_);
  debug::destroy_vk_callbacks();
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
  if (ghost_window_) {
    VkImage image; /* TODO will be used for reading later... */
    VkFramebuffer framebuffer;
    VkRenderPass render_pass;
    VkExtent2D extent;
    uint32_t fb_id;

    GHOST_GetVulkanBackbuffer(
        (GHOST_WindowHandle)ghost_window_, &image, &framebuffer, &render_pass, &extent, &fb_id);

    /* Recreate the gpu::VKFrameBuffer wrapper after every swap. */
    delete back_left;

    back_left = new VKFrameBuffer("back_left", framebuffer, render_pass, extent);
    active_fb = back_left;
  }
}

void VKContext::deactivate()
{
}

void VKContext::begin_frame()
{
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

void VKContext::memory_statistics_get(int * /*total_mem*/, int * /*free_mem*/)
{
}

void VKContext::debug_group_begin(const char *label, int /* id */)
{
  debug::pushMarker(vk_queue_,label);
}

void VKContext::debug_group_end()
{
  debug::popMarker(vk_queue_);
}
}  // namespace blender::gpu


namespace blender::gpu{
#ifdef VK_NO_PROTOTYPES
  VKWrapper vk_wrapper;
#endif
  VkResult VKFunctionsLoader::vulkan_dynamic_load(void)
  {
#ifdef VK_NO_PROTOTYPES
#  if defined(_WIN32)
    HMODULE vulkanDll = LoadLibraryA("vulkan-1.dll");
    if (!vulkanDll) {
      return VK_ERROR_INITIALIZATION_FAILED;
    }
#  elif defined(__APPLE__)
    void *vulkanDll = dlopen("libvulkan.dylib", RTLD_NOW | RTLD_LOCAL);
    if (!vulkanDll) {
      vulkanDll = dlopen("libvulkan.1.dylib", RTLD_NOW | RTLD_LOCAL);
    }
    if (!vulkanDll) {
      vulkanDll = dlopen("libMoltenVK.dylib", RTLD_NOW | RTLD_LOCAL);
    }
    if (!vulkanDll) {
      return VK_ERROR_INITIALIZATION_FAILED;
    }

#  else
    void *vulkanDll = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!vulkanDll) {
      vulkanDll = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
    }
    if (!vulkanDll) {
      return VK_ERROR_INITIALIZATION_FAILED;
    }
#  endif

#  define LOAD(name) GET_FUNC_ADDRESS(vulkanDll, name)

    vk_wrapper.vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)LOAD("vkGetInstanceProcAddr");
    vk_wrapper.vkCreateInstance_r = (PFN_vkCreateInstance)LOAD("vkCreateInstance");
    vk_wrapper.vkDestroyInstance = (PFN_vkDestroyInstance)LOAD("vkDestroyInstance");

    vk_wrapper.vkGetPhysicalDeviceImageFormatProperties_r =
        (PFN_vkGetPhysicalDeviceImageFormatProperties)LOAD(
            "vkGetPhysicalDeviceImageFormatProperties");
    vk_wrapper.vkGetPhysicalDeviceProperties = (PFN_vkGetPhysicalDeviceProperties)LOAD(
        "vkGetPhysicalDeviceProperties");

#  undef LOAD
#endif
    return VK_SUCCESS;
  }

  void VKFunctionsLoader::vulkan_dynamic_load_instance(VkInstance &instance)
  {
#ifdef VK_NO_PROTOTYPES
#  define INSTLOAD vk_wrapper.vkGetInstanceProcAddr
    vk_wrapper.vkGetDeviceProcAddr = (PFN_vkGetDeviceProcAddr)INSTLOAD(instance,"vkGetDeviceProcAddr");
#else
#  define INSTLOAD vkGetInstanceProcAddr
#endif
    debug::init_vk_callbacks(instance, INSTLOAD);
#undef INSTLOAD
  }

  void VKFunctionsLoader::vulkan_dynamic_load_device(VkDevice &device)
  {
#ifdef VK_NO_PROTOTYPES
#  define DEVLOAD(name) vk_wrapper.vkGetDeviceProcAddr(device, name)
#else
#  define DEVLOAD(name) vkGetDeviceProcAddr(device, name)
#endif
    /*There are currently no extension functions to load.*/
#ifdef VK_NO_PROTOTYPES
    vk_wrapper.vkAllocateCommandBuffers_r = (PFN_vkAllocateCommandBuffers)DEVLOAD(
        "vkAllocateCommandBuffers");
    vk_wrapper.vkAllocateDescriptorSets_r = (PFN_vkAllocateDescriptorSets)DEVLOAD(
        "vkAllocateDescriptorSets");

    vk_wrapper.vkCmdBindDescriptorSets = (PFN_vkCmdBindDescriptorSets)DEVLOAD(
        "vkCmdBindDescriptorSets");
    vk_wrapper.vkCmdBindPipeline = (PFN_vkCmdBindPipeline)DEVLOAD("vkCmdBindPipeline");
    vk_wrapper.vkCmdCopyImageToBuffer = (PFN_vkCmdCopyImageToBuffer)DEVLOAD(
        "vkCmdCopyImageToBuffer");
    vk_wrapper.vkCmdDispatch = (PFN_vkCmdDispatch)DEVLOAD("vkCmdDispatch");
    vk_wrapper.vkCmdPipelineBarrier = (PFN_vkCmdPipelineBarrier)DEVLOAD("vkCmdPipelineBarrier");
    vk_wrapper.vkCmdPushConstants = (PFN_vkCmdPushConstants)DEVLOAD("vkCmdPushConstants");

    vk_wrapper.vkCreateComputePipelines_r = (PFN_vkCreateComputePipelines)DEVLOAD(
        "vkCreateComputePipelines");
    vk_wrapper.vkCreateDescriptorPool_r = (PFN_vkCreateDescriptorPool)DEVLOAD(
        "vkCreateDescriptorPool");
    vk_wrapper.vkCreateDescriptorSetLayout_r = (PFN_vkCreateDescriptorSetLayout)DEVLOAD(
        "vkCreateDescriptorSetLayout");
    vk_wrapper.vkCreateFence_r = (PFN_vkCreateFence)DEVLOAD("vkCreateFence");
    vk_wrapper.vkCreateFramebuffer_r = (PFN_vkCreateFramebuffer)DEVLOAD("vkCreateFramebuffer");
    vk_wrapper.vkCreateImage_r = (PFN_vkCreateImage)DEVLOAD("vkCreateImage");
    vk_wrapper.vkCreateImageView_r = (PFN_vkCreateImageView)DEVLOAD("vkCreateImageView");
    vk_wrapper.vkCreatePipelineLayout_r = (PFN_vkCreatePipelineLayout)DEVLOAD(
        "vkCreatePipelineLayout");
    vk_wrapper.vkCreateShaderModule_r = (PFN_vkCreateShaderModule)DEVLOAD("vkCreateShaderModule");

    vk_wrapper.vkDestroyDescriptorPool = (PFN_vkDestroyDescriptorPool)DEVLOAD(
        "vkDestroyDescriptorPool");
    vk_wrapper.vkDestroyDescriptorSetLayout = (PFN_vkDestroyDescriptorSetLayout)DEVLOAD(
        "vkDestroyDescriptorSetLayout");
    vk_wrapper.vkDestroyFence = (PFN_vkDestroyFence)DEVLOAD("vkDestroyFence");
    vk_wrapper.vkDestroyFramebuffer = (PFN_vkDestroyFramebuffer)DEVLOAD("vkDestroyFramebuffer");
    vk_wrapper.vkDestroyImage = (PFN_vkDestroyImage)DEVLOAD("vkDestroyImage");
    vk_wrapper.vkDestroyImageView = (PFN_vkDestroyImageView)DEVLOAD("vkDestroyImageView");
    vk_wrapper.vkDestroyPipeline = (PFN_vkDestroyPipeline)DEVLOAD("vkDestroyPipeline");
    vk_wrapper.vkDestroyPipelineLayout = (PFN_vkDestroyPipelineLayout)DEVLOAD(
        "vkDestroyPipelineLayout");
    vk_wrapper.vkDestroyShaderModule = (PFN_vkDestroyShaderModule)DEVLOAD("vkDestroyShaderModule");

    vk_wrapper.vkFreeCommandBuffers = (PFN_vkFreeCommandBuffers)DEVLOAD("vkFreeCommandBuffers");
    vk_wrapper.vkFreeDescriptorSets_r = (PFN_vkFreeDescriptorSets)DEVLOAD("vkFreeDescriptorSets");
    vk_wrapper.vkQueueSubmit_r = (PFN_vkQueueSubmit)DEVLOAD("vkQueueSubmit");

    vk_wrapper.vkResetCommandBuffer_r = (PFN_vkResetCommandBuffer)DEVLOAD("vkResetCommandBuffer");
    vk_wrapper.vkResetFences_r = (PFN_vkResetFences)DEVLOAD("vkResetFences");

    vk_wrapper.vkBeginCommandBuffer_r = (PFN_vkBeginCommandBuffer)DEVLOAD("vkBeginCommandBuffer");
    vk_wrapper.vkEndCommandBuffer_r = (PFN_vkEndCommandBuffer)DEVLOAD("vkEndCommandBuffer");

    vk_wrapper.vkUpdateDescriptorSets = (PFN_vkUpdateDescriptorSets)DEVLOAD(
        "vkUpdateDescriptorSets");
    vk_wrapper.vkWaitForFences_r = (PFN_vkWaitForFences)DEVLOAD("vkWaitForFences");
#else
    (void *)device;
#endif

#undef DEVLOAD
  }

}  // namespace blender::gpu
