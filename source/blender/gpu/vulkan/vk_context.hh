/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_context_private.hh"
#include "vk_debug.hh"
#include "vk_command_buffer.hh"
#include "vk_descriptor_pools.hh"


namespace blender::gpu {
class VKContext : public Context {
 private:
  /** Copies of the handles owned by the GHOST context. */
  VkInstance vk_instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice vk_physical_device_ = VK_NULL_HANDLE;
  VkDevice vk_device_ = VK_NULL_HANDLE;
  VKCommandBuffer command_buffer_;
  uint32_t vk_queue_family_ = 0;
  VkQueue vk_queue_ = VK_NULL_HANDLE;

  /** Allocator used for texture and buffers and other resources. */
  VmaAllocator mem_allocator_ = VK_NULL_HANDLE;
  VKDescriptorPools descriptor_pools_;

  /** Limits of the device linked to this context. */
  VkPhysicalDeviceLimits vk_physical_device_limits_;

  void *ghost_context_;

 public:
  VKContext(void *ghost_window, void *ghost_context);
  virtual ~VKContext();

  void activate() override;
  void deactivate() override;
  void begin_frame() override;
  void end_frame() override;

  void flush() override;
  void finish() override;

  void memory_statistics_get(int *total_mem, int *free_mem) override;

  void debug_group_begin(const char *, int) override;
  void debug_group_end() override;

  static VKContext *get(void)
  {
    return static_cast<VKContext *>(Context::get());
  }

  VkPhysicalDevice physical_device_get() const
  {
    return vk_physical_device_;
  }

  const VkPhysicalDeviceLimits &physical_device_limits_get() const
  {
    return vk_physical_device_limits_;
  }

  VkDevice device_get() const
  {
    return vk_device_;
  }

  VKCommandBuffer &command_buffer_get()
  {
    return command_buffer_;
  }

  VkQueue queue_get() const
  {
    return vk_queue_;
  }

  const uint32_t *queue_family_ptr_get() const
  {
    return &vk_queue_family_;
  }

  VKDescriptorPools &descriptor_pools_get()
  {
    return descriptor_pools_;
  }

  VmaAllocator mem_allocator_get() const
  {
    return mem_allocator_;
  }

 private:
  void init_physical_device_limits();
};

}  // namespace blender::gpu


namespace blender::gpu {

#ifdef VK_NO_PROTOTYPES
/**
 * Inside the GPU module, compile nullptrs and load them outside.
 * Wrap them in externally defined vulkan functions.
 * Struct to declare  `PFN_vk* functions` just we need.
 **/
struct VKWrapper {

  PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr;
  PFN_vkGetDeviceProcAddr vkGetDeviceProcAddr;

  PFN_vkCreateInstance vkCreateInstance_r;
  PFN_vkDestroyInstance vkDestroyInstance;

  PFN_vkCreateImage vkCreateImage_r;
  PFN_vkDestroyImage vkDestroyImage;

  PFN_vkCreateImageView vkCreateImageView_r;
  PFN_vkDestroyImageView vkDestroyImageView;

  PFN_vkCreateDescriptorPool vkCreateDescriptorPool_r;
  PFN_vkDestroyDescriptorPool vkDestroyDescriptorPool;

  PFN_vkCreateDescriptorSetLayout vkCreateDescriptorSetLayout_r;
  PFN_vkDestroyDescriptorSetLayout vkDestroyDescriptorSetLayout;

  PFN_vkCreatePipelineLayout vkCreatePipelineLayout_r;
  PFN_vkDestroyPipelineLayout vkDestroyPipelineLayout;

  PFN_vkCreateFence vkCreateFence_r;
  PFN_vkDestroyFence vkDestroyFence;
  PFN_vkResetFences vkResetFences_r;
  PFN_vkWaitForFences vkWaitForFences_r;

  PFN_vkCreateComputePipelines vkCreateComputePipelines_r;
  PFN_vkDestroyPipeline vkDestroyPipeline;

  PFN_vkCreateShaderModule vkCreateShaderModule_r;
  PFN_vkDestroyShaderModule vkDestroyShaderModule;

  PFN_vkCreateFramebuffer vkCreateFramebuffer_r;
  PFN_vkDestroyFramebuffer vkDestroyFramebuffer;

  PFN_vkAllocateCommandBuffers vkAllocateCommandBuffers_r;
  PFN_vkFreeCommandBuffers vkFreeCommandBuffers;
  PFN_vkResetCommandBuffer vkResetCommandBuffer_r;

  PFN_vkBeginCommandBuffer vkBeginCommandBuffer_r;
  PFN_vkEndCommandBuffer vkEndCommandBuffer_r;

  PFN_vkCmdBindDescriptorSets vkCmdBindDescriptorSets;
  PFN_vkCmdBindPipeline vkCmdBindPipeline;
  PFN_vkCmdPushConstants vkCmdPushConstants;
  PFN_vkCmdCopyImageToBuffer vkCmdCopyImageToBuffer;
  PFN_vkCmdPipelineBarrier vkCmdPipelineBarrier;
  PFN_vkCmdDispatch vkCmdDispatch;

  PFN_vkAllocateDescriptorSets vkAllocateDescriptorSets_r;
  PFN_vkFreeDescriptorSets vkFreeDescriptorSets_r;
  PFN_vkUpdateDescriptorSets vkUpdateDescriptorSets;

  PFN_vkQueueSubmit vkQueueSubmit_r;

  PFN_vkGetPhysicalDeviceProperties vkGetPhysicalDeviceProperties;
  PFN_vkGetPhysicalDeviceImageFormatProperties vkGetPhysicalDeviceImageFormatProperties_r;
};
#  ifdef VK_WRAPPER_IMPL
extern VKWrapper vk_wrapper;
#  endif
#endif
class VKFunctionsLoader {
 public:
  /**
   * Here we load all the vulkan functions.
   *
   * Keep only what blender uses.
   *
   * Flow of dynamically loading vulkan functions.
   *
   * As a premise, define #NO_PROTOTYPES before including `vulkan.h`.
   *
   * Here, LibraryFunc (LF) is a function that gets a library such as LoadlibraryA.
   *
   * 0.  Get LF1 per OS.
   * 1.  Load vulkan dll(DLL1) from LF1.
   * 2.  Retrieves the built-in functions and LF2 from DLL1. (#vulkan_dynamic_load)
   **/
  VkResult vulkan_dynamic_load(void);

  /**
   * 1.  Load instance dll(DLL2) from LF2.
   * 2.  Retrieves the instance extension functions which  must be enabled and LF3 from DLL2.
   * (#vulkan_dynamic_load_instance).
   */
  void vulkan_dynamic_load_instance(VkInstance &instance);

  /**
   *
   * 1.  Load device dll(DLL3) dll from LF3.
   * 2.  Retrieves the device extension functions which  must be enabled from DLL3.
   * (#vulkan_dynamic_load_device).
   */
  void vulkan_dynamic_load_device(VkDevice &device);
};
}  // namespace blender::gpu

#ifdef VK_NO_PROTOTYPES

/* clang-format off */
#ifndef VK_WRAPPER_IMPL
#define VK_WRAPPER_PREFIX inline
#define VK_FUNC_ERROR_CHECK_DEFINE(func, ...) \
VK_WRAPPER_PREFIX  VKAPI_ATTR VkResult VKAPI_CALL func(ARG_LIST(__VA_ARGS__)); 

#define VK_FUNC_VOID_DEFINE(func, ...) \
VK_WRAPPER_PREFIX  VKAPI_ATTR void VKAPI_CALL func(ARG_LIST(__VA_ARGS__)); 

#else

inline  void VK_ERROR_CHECK(VkResult r, const char *name);


#define VK_WRAPPER_PREFIX extern inline
#  define VK_FUNC_ERROR_CHECK_DEFINE(func, ...) \
VK_WRAPPER_PREFIX VKAPI_ATTR VkResult VKAPI_CALL func(ARG_LIST(__VA_ARGS__)) \
{ \
  VkResult r = blender::gpu::vk_wrapper.##func##_r(ARG_LIST_CALL(__VA_ARGS__)); \
  if (G.debug & G_DEBUG_GPU) { \
    VK_ERROR_CHECK(r, __STR_VK_CHECK(func(__VA_ARGS__))); \
  } \
  return r; \
};

#  define VK_FUNC_VOID_DEFINE(func, ...) \
VK_WRAPPER_PREFIX VKAPI_ATTR void VKAPI_CALL func(ARG_LIST(__VA_ARGS__)) \
{ \
  blender::gpu::vk_wrapper.func(ARG_LIST_CALL(__VA_ARGS__)); \
};
#endif

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateInstance,const VkInstanceCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkInstance*, pInstance);
VK_FUNC_VOID_DEFINE(vkDestroyInstance,VkInstance, instance,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateImageView , VkDevice , device, const VkImageViewCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkImageView* ,pView);
VK_FUNC_VOID_DEFINE(vkDestroyImageView,VkDevice ,device,VkImageView, imageView,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateDescriptorSetLayout , VkDevice , device, const VkDescriptorSetLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorSetLayout* ,pSetLayout);
VK_FUNC_VOID_DEFINE(vkDestroyDescriptorSetLayout,VkDevice ,device,VkDescriptorSetLayout, descriptorSetLayout,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateDescriptorPool , VkDevice , device, const VkDescriptorPoolCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorPool*, pDescriptorPool);
VK_FUNC_VOID_DEFINE(vkDestroyDescriptorPool,VkDevice ,device,VkDescriptorPool ,descriptorPool,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreatePipelineLayout , VkDevice , device, const VkPipelineLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkPipelineLayout* ,pPipelineLayout);
VK_FUNC_VOID_DEFINE(vkDestroyPipelineLayout,VkDevice ,device,VkPipelineLayout ,pipelineLayout,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateFence , VkDevice ,device, const VkFenceCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFence* ,pFence);
VK_FUNC_VOID_DEFINE(vkDestroyFence,VkDevice, device,VkFence, fence,const VkAllocationCallbacks *,pAllocator);
VK_FUNC_ERROR_CHECK_DEFINE(vkWaitForFences , VkDevice , device, uint32_t ,fenceCount, const VkFence*, pFences, VkBool32 ,waitAll, uint64_t, timeout);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetFences , VkDevice, device, uint32_t ,fenceCount, const VkFence* ,pFences);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateComputePipelines , VkDevice , device, VkPipelineCache ,pipelineCache, uint32_t ,createInfoCount, const VkComputePipelineCreateInfo*, pCreateInfos, const VkAllocationCallbacks* ,pAllocator, VkPipeline* ,pPipelines);
VK_FUNC_VOID_DEFINE(vkDestroyPipeline,VkDevice ,device,VkPipeline ,pipeline,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateShaderModule , VkDevice , device, const VkShaderModuleCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkShaderModule* ,pShaderModule);
VK_FUNC_VOID_DEFINE(vkDestroyShaderModule,VkDevice ,device,VkShaderModule ,shaderModule,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkCreateFramebuffer , VkDevice , device, const VkFramebufferCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFramebuffer* ,pFramebuffer);
VK_FUNC_VOID_DEFINE(vkDestroyFramebuffer,VkDevice ,device,VkFramebuffer ,framebuffer,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_ERROR_CHECK_DEFINE(vkAllocateCommandBuffers , VkDevice , device, const VkCommandBufferAllocateInfo* ,pAllocateInfo, VkCommandBuffer* ,pCommandBuffers);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetCommandBuffer , VkCommandBuffer ,commandBuffer, VkCommandBufferResetFlags, flags);
VK_FUNC_ERROR_CHECK_DEFINE(vkBeginCommandBuffer , VkCommandBuffer, commandBuffer, const VkCommandBufferBeginInfo*, pBeginInfo);
VK_FUNC_ERROR_CHECK_DEFINE(vkEndCommandBuffer , VkCommandBuffer ,commandBuffer);

VK_FUNC_VOID_DEFINE(vkCmdBindPipeline,VkCommandBuffer, commandBuffer,VkPipelineBindPoint ,pipelineBindPoint,VkPipeline ,pipeline);
VK_FUNC_VOID_DEFINE(vkCmdBindDescriptorSets,VkCommandBuffer ,commandBuffer,VkPipelineBindPoint ,pipelineBindPoint,VkPipelineLayout, layout,uint32_t ,firstSet,uint32_t ,descriptorSetCount,const VkDescriptorSet *,pDescriptorSets,uint32_t ,dynamicOffsetCount,const uint32_t *,pDynamicOffsets);
VK_FUNC_VOID_DEFINE(vkCmdPushConstants,VkCommandBuffer ,commandBuffer,VkPipelineLayout ,layout,VkShaderStageFlags, stageFlags,uint32_t ,offset,uint32_t ,size,const void *,pValues);
VK_FUNC_VOID_DEFINE(vkCmdCopyImageToBuffer,VkCommandBuffer, commandBuffer,VkImage ,srcImage,VkImageLayout ,srcImageLayout,VkBuffer ,dstBuffer,uint32_t ,regionCount,const VkBufferImageCopy *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdPipelineBarrier,VkCommandBuffer ,commandBuffer,VkPipelineStageFlags ,srcStageMask,VkPipelineStageFlags ,dstStageMask,VkDependencyFlags ,dependencyFlags,uint32_t ,memoryBarrierCount,const VkMemoryBarrier *,pMemoryBarriers,uint32_t ,bufferMemoryBarrierCount,const VkBufferMemoryBarrier *,pBufferMemoryBarriers,uint32_t ,imageMemoryBarrierCount,const VkImageMemoryBarrier *,pImageMemoryBarriers);
VK_FUNC_VOID_DEFINE(vkCmdDispatch,VkCommandBuffer, commandBuffer,uint32_t ,groupCountX,uint32_t ,groupCountY,uint32_t ,groupCountZ);

VK_FUNC_ERROR_CHECK_DEFINE(vkAllocateDescriptorSets , VkDevice , device, const VkDescriptorSetAllocateInfo* ,pAllocateInfo, VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_ERROR_CHECK_DEFINE(vkFreeDescriptorSets , VkDevice , device, VkDescriptorPool, descriptorPool, uint32_t ,descriptorSetCount, const VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_VOID_DEFINE(vkUpdateDescriptorSets,VkDevice ,device,uint32_t ,descriptorWriteCount,const VkWriteDescriptorSet *,pDescriptorWrites,uint32_t ,descriptorCopyCount,const VkCopyDescriptorSet *,pDescriptorCopies);


VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceProperties,VkPhysicalDevice ,physicalDevice, VkPhysicalDeviceProperties *,pProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetPhysicalDeviceImageFormatProperties , VkPhysicalDevice ,physicalDevice, VkFormat ,format, VkImageType ,type, VkImageTiling, tiling, VkImageUsageFlags, usage, VkImageCreateFlags ,flags, VkImageFormatProperties* ,pImageFormatProperties);

VK_FUNC_ERROR_CHECK_DEFINE(vkQueueSubmit , VkQueue, queue, uint32_t, submitCount, const VkSubmitInfo*, pSubmits, VkFence ,fence);

/* clang-format on */

#endif
