/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 *
 * Debug features of Vulkan.
 */

#include "BLI_set.hh"
#include "BLI_system.h"
#include  "BKE_global.h"
#include  "CLG_log.h"
#include  "GPU_debug.h"
#include  "GPU_platform.h"



#include  "vk_common.hh"
#include  "vk_debug.hh"
#include  "vk_context.hh"


#if defined(__unix__) || defined(__APPLE__)
#  include <sys/time.h>
#  include <unistd.h>
#  define GetFuncAddress dlsym
#endif

#if defined(_MSC_VER)
#  include <Windows.h>

#  include <VersionHelpers.h> /* This needs to be included after Windows.h. */
#  include <io.h>
#  if !defined(ENABLE_VIRTUAL_TERMINAL_PROCESSING)
#    define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#  endif

 #  define GetFuncAddress (void *)GetProcAddress
#endif








static CLG_LogRef LOG = {"gpu.debug.vulkan"};

/* Function pointer definitions for use only in this file.*/
#if defined(VK_EXT_debug_utils)
PFN_vkCreateDebugUtilsMessengerEXT  pfvkCreateDebugUtilsMessengerEXT  = nullptr;
PFN_vkDestroyDebugUtilsMessengerEXT pfvkDestroyDebugUtilsMessengerEXT = nullptr;
PFN_vkSubmitDebugUtilsMessageEXT pfvkSubmitDebugUtilsMessageEXT = nullptr;
PFN_vkCmdBeginDebugUtilsLabelEXT pfvkCmdBeginDebugUtilsLabelEXT =nullptr;
PFN_vkCmdEndDebugUtilsLabelEXT pfvkCmdEndDebugUtilsLabelEXT = nullptr;
PFN_vkCmdInsertDebugUtilsLabelEXT pfvkCmdInsertDebugUtilsLabelEXT = nullptr;
PFN_vkQueueBeginDebugUtilsLabelEXT pfvkQueueBeginDebugUtilsLabelEXT = nullptr;
PFN_vkQueueEndDebugUtilsLabelEXT pfvkQueueEndDebugUtilsLabelEXT = nullptr;
PFN_vkQueueInsertDebugUtilsLabelEXT pfvkQueueInsertDebugUtilsLabelEXT = nullptr;
PFN_vkSetDebugUtilsObjectNameEXT pfvkSetDebugUtilsObjectNameEXT = nullptr;
PFN_vkSetDebugUtilsObjectTagEXT pfvkSetDebugUtilsObjectTagEXT = nullptr;
#endif /* defined(VK_EXT_debug_utils) */

struct VKWrapper {
  PFN_vkAllocateCommandBuffers pfvkAllocateCommandBuffers;
  PFN_vkAllocateDescriptorSets pfvkAllocateDescriptorSets;
  PFN_vkAllocateMemory pfvkAllocateMemory;
  PFN_vkBeginCommandBuffer pfvkBeginCommandBuffer;
  PFN_vkBindBufferMemory pfvkBindBufferMemory;
  PFN_vkBindImageMemory pfvkBindImageMemory;
  PFN_vkCmdBeginQuery vkCmdBeginQuery;
  PFN_vkCmdBeginRenderPass vkCmdBeginRenderPass;
  PFN_vkCmdBindDescriptorSets vkCmdBindDescriptorSets;
  PFN_vkCmdBindIndexBuffer vkCmdBindIndexBuffer;
  PFN_vkCmdBindPipeline vkCmdBindPipeline;
  PFN_vkCmdBindVertexBuffers vkCmdBindVertexBuffers;
  PFN_vkCmdBlitImage vkCmdBlitImage;
  PFN_vkCmdClearAttachments vkCmdClearAttachments;
  PFN_vkCmdClearColorImage vkCmdClearColorImage;
  PFN_vkCmdClearDepthStencilImage vkCmdClearDepthStencilImage;
  PFN_vkCmdCopyBuffer vkCmdCopyBuffer;
  PFN_vkCmdCopyBufferToImage vkCmdCopyBufferToImage;
  PFN_vkCmdCopyImage vkCmdCopyImage;
  PFN_vkCmdCopyImageToBuffer vkCmdCopyImageToBuffer;
  PFN_vkCmdCopyQueryPoolResults vkCmdCopyQueryPoolResults;
  PFN_vkCmdDispatch vkCmdDispatch;
  PFN_vkCmdDispatchIndirect vkCmdDispatchIndirect;
  PFN_vkCmdDraw vkCmdDraw;
  PFN_vkCmdDrawIndexed vkCmdDrawIndexed;
  PFN_vkCmdDrawIndexedIndirect vkCmdDrawIndexedIndirect;
  PFN_vkCmdDrawIndirect vkCmdDrawIndirect;
  PFN_vkCmdEndQuery vkCmdEndQuery;
  PFN_vkCmdEndRenderPass vkCmdEndRenderPass;
  PFN_vkCmdExecuteCommands vkCmdExecuteCommands;
  PFN_vkCmdFillBuffer vkCmdFillBuffer;
  PFN_vkCmdNextSubpass vkCmdNextSubpass;
  PFN_vkCmdPipelineBarrier vkCmdPipelineBarrier;
  PFN_vkCmdPushConstants vkCmdPushConstants;
  PFN_vkCmdResetEvent vkCmdResetEvent;
  PFN_vkCmdResetQueryPool vkCmdResetQueryPool;
  PFN_vkCmdResolveImage vkCmdResolveImage;
  PFN_vkCmdSetBlendConstants vkCmdSetBlendConstants;
  PFN_vkCmdSetDepthBias vkCmdSetDepthBias;
  PFN_vkCmdSetDepthBounds vkCmdSetDepthBounds;
  PFN_vkCmdSetEvent vkCmdSetEvent;
  PFN_vkCmdSetLineWidth vkCmdSetLineWidth;
  PFN_vkCmdSetScissor vkCmdSetScissor;
  PFN_vkCmdSetStencilCompareMask vkCmdSetStencilCompareMask;
  PFN_vkCmdSetStencilReference vkCmdSetStencilReference;
  PFN_vkCmdSetStencilWriteMask vkCmdSetStencilWriteMask;
  PFN_vkCmdSetViewport vkCmdSetViewport;
  PFN_vkCmdUpdateBuffer vkCmdUpdateBuffer;
  PFN_vkCmdWaitEvents vkCmdWaitEvents;
  PFN_vkCmdWriteTimestamp vkCmdWriteTimestamp;
  PFN_vkCreateBuffer pfvkCreateBuffer;
  PFN_vkCreateBufferView pfvkCreateBufferView;
  PFN_vkCreateCommandPool pfvkCreateCommandPool;
  PFN_vkCreateComputePipelines pfvkCreateComputePipelines;
  PFN_vkCreateDescriptorPool pfvkCreateDescriptorPool;
  PFN_vkCreateDescriptorSetLayout pfvkCreateDescriptorSetLayout;
  PFN_vkCreateDevice pfvkCreateDevice;
  PFN_vkCreateEvent pfvkCreateEvent;
  PFN_vkCreateFence pfvkCreateFence;
  PFN_vkCreateFramebuffer pfvkCreateFramebuffer;
  PFN_vkCreateGraphicsPipelines pfvkCreateGraphicsPipelines;
  PFN_vkCreateImage pfvkCreateImage;
  PFN_vkCreateImageView pfvkCreateImageView;
  PFN_vkCreateInstance pfvkCreateInstance;
  PFN_vkCreatePipelineCache pfvkCreatePipelineCache;
  PFN_vkCreatePipelineLayout pfvkCreatePipelineLayout;
  PFN_vkCreateQueryPool pfvkCreateQueryPool;
  PFN_vkCreateRenderPass pfvkCreateRenderPass;
  PFN_vkCreateSampler pfvkCreateSampler;
  PFN_vkCreateSemaphore pfvkCreateSemaphore;
  PFN_vkCreateShaderModule pfvkCreateShaderModule;
  PFN_vkDestroyBuffer vkDestroyBuffer;
  PFN_vkDestroyBufferView vkDestroyBufferView;
  PFN_vkDestroyCommandPool vkDestroyCommandPool;
  PFN_vkDestroyDescriptorPool vkDestroyDescriptorPool;
  PFN_vkDestroyDescriptorSetLayout vkDestroyDescriptorSetLayout;
  PFN_vkDestroyDevice vkDestroyDevice;
  PFN_vkDestroyEvent vkDestroyEvent;
  PFN_vkDestroyFence vkDestroyFence;
  PFN_vkDestroyFramebuffer vkDestroyFramebuffer;
  PFN_vkDestroyImage vkDestroyImage;
  PFN_vkDestroyImageView vkDestroyImageView;
  PFN_vkDestroyInstance vkDestroyInstance;
  PFN_vkDestroyPipeline vkDestroyPipeline;
  PFN_vkDestroyPipelineCache vkDestroyPipelineCache;
  PFN_vkDestroyPipelineLayout vkDestroyPipelineLayout;
  PFN_vkDestroyQueryPool vkDestroyQueryPool;
  PFN_vkDestroyRenderPass vkDestroyRenderPass;
  PFN_vkDestroySampler vkDestroySampler;
  PFN_vkDestroySemaphore vkDestroySemaphore;
  PFN_vkDestroyShaderModule vkDestroyShaderModule;
  PFN_vkDeviceWaitIdle pfvkDeviceWaitIdle;
  PFN_vkEndCommandBuffer pfvkEndCommandBuffer;
  PFN_vkEnumerateDeviceExtensionProperties pfvkEnumerateDeviceExtensionProperties;
  PFN_vkEnumerateDeviceLayerProperties pfvkEnumerateDeviceLayerProperties;
  PFN_vkEnumerateInstanceExtensionProperties pfvkEnumerateInstanceExtensionProperties;
  PFN_vkEnumerateInstanceLayerProperties pfvkEnumerateInstanceLayerProperties;
  PFN_vkEnumeratePhysicalDevices pfvkEnumeratePhysicalDevices;
  PFN_vkFlushMappedMemoryRanges pfvkFlushMappedMemoryRanges;
  PFN_vkFreeCommandBuffers vkFreeCommandBuffers;
  PFN_vkFreeDescriptorSets pfvkFreeDescriptorSets;
  PFN_vkFreeMemory vkFreeMemory;

  PFN_vkGetBufferMemoryRequirements vkGetBufferMemoryRequirements;
  PFN_vkGetDeviceMemoryCommitment vkGetDeviceMemoryCommitment;
  PFN_vkGetDeviceProcAddr vkGetDeviceProcAddr;
  PFN_vkGetDeviceQueue vkGetDeviceQueue;
  PFN_vkGetEventStatus pfvkGetEventStatus;
  PFN_vkGetFenceStatus pfvkGetFenceStatus;
  PFN_vkGetImageMemoryRequirements vkGetImageMemoryRequirements;
  PFN_vkGetImageSparseMemoryRequirements vkGetImageSparseMemoryRequirements;
  PFN_vkGetImageSubresourceLayout vkGetImageSubresourceLayout;
  PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr;
  PFN_vkGetPhysicalDeviceFeatures vkGetPhysicalDeviceFeatures;
  PFN_vkGetPhysicalDeviceFormatProperties vkGetPhysicalDeviceFormatProperties;
  PFN_vkGetPhysicalDeviceImageFormatProperties pfvkGetPhysicalDeviceImageFormatProperties;
  PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties;
  PFN_vkGetPhysicalDeviceProperties vkGetPhysicalDeviceProperties;
  PFN_vkGetPhysicalDeviceQueueFamilyProperties vkGetPhysicalDeviceQueueFamilyProperties;
  PFN_vkGetPhysicalDeviceSparseImageFormatProperties vkGetPhysicalDeviceSparseImageFormatProperties;
  PFN_vkGetPipelineCacheData pfvkGetPipelineCacheData;
  PFN_vkGetQueryPoolResults pfvkGetQueryPoolResults;
  PFN_vkGetRenderAreaGranularity vkGetRenderAreaGranularity;
  PFN_vkInvalidateMappedMemoryRanges pfvkInvalidateMappedMemoryRanges;
  PFN_vkMapMemory pfvkMapMemory;
  PFN_vkMergePipelineCaches pfvkMergePipelineCaches;
  PFN_vkQueueBindSparse pfvkQueueBindSparse;
  PFN_vkQueueSubmit pfvkQueueSubmit;
  PFN_vkQueueWaitIdle pfvkQueueWaitIdle;
  PFN_vkResetCommandBuffer pfvkResetCommandBuffer;
  PFN_vkResetCommandPool pfvkResetCommandPool;
  PFN_vkResetDescriptorPool pfvkResetDescriptorPool;
  PFN_vkResetEvent pfvkResetEvent;
  PFN_vkResetFences pfvkResetFences;
  PFN_vkSetEvent pfvkSetEvent;
  PFN_vkUnmapMemory vkUnmapMemory;
  PFN_vkUpdateDescriptorSets vkUpdateDescriptorSets;
  PFN_vkWaitForFences pfvkWaitForFences;
};



namespace blender {
namespace gpu {
namespace debug {
struct VKDebuggingTools {
  bool enabled = false;
  /*One -on -one for instance.*/
  VkInstance  instance = VK_NULL_HANDLE;
  VkDevice     device = VK_NULL_HANDLE;
  VKWrapper vk_wrapper;
  VkDebugUtilsMessengerEXT dbgMessenger = nullptr;
  Set<int32_t> dbgIgnoreMessages;
  std::mutex lists_mutex_;

  VKDebuggingTools()
  {
    clear();
  }
  void clear()
  {
    instance = VK_NULL_HANDLE;
    device = VK_NULL_HANDLE;
    dbgIgnoreMessages.clear();
    dbgMessenger = nullptr;
    enabled = false;
  }
  void add_ignore(int32_t id)
  {
    lists_mutex_.lock();
    dbgIgnoreMessages.add(id);
    lists_mutex_.unlock();
  }
  void remove_ignore(int32_t id)
  {
    lists_mutex_.lock();
    dbgIgnoreMessages.remove(id);
    lists_mutex_.unlock();
  }


};
static VKDebuggingTools tools;
}  // namespace debug
}  // namespace gpu
}  // namespace blender

  /*If we don't have multiple instances, VKDebuggingTools is Singleton.*/

#define __STR_VK_CHECK(s) "" #s

#define VK_ERROR_CHECK(r, name) \
  { \
    if (G.debug & G_DEBUG_GPU) { \
      if (r != VK_SUCCESS) { \
        fprintf(stderr, \
                "Vulkan Error :: %s failled with %s\n", \
                name, \
                blender::gpu::to_vk_error_string(r)); \
      } \
    } \
  }


#define VK_FUNC_ERROR_CHECK_WRAPPER(func, ...) \
VKAPI_ATTR VkResult VKAPI_CALL func(ARG_LIST(__VA_ARGS__)) \
  { \
    VkResult r = blender::gpu::debug::tools.vk_wrapper.pf##func(ARG_LIST_CALL(__VA_ARGS__)); \
    VK_ERROR_CHECK(r,__STR_VK_CHECK(func(__VA_ARGS__)));\
    return r;\
  };

#define VK_FUNC_VOID_WRAPPER(func, ...) \
VKAPI_ATTR void VKAPI_CALL func(ARG_LIST(__VA_ARGS__)) \
  {  blender::gpu::debug::tools.vk_wrapper.func(ARG_LIST_CALL(__VA_ARGS__)); };


VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateInstance,
                            const VkInstanceCreateInfo *,
                            pCreateInfo,
                            const VkAllocationCallbacks *,
                            pAllocator,
                            VkInstance *,
                            pInstance);

VK_FUNC_ERROR_CHECK_WRAPPER(vkEnumeratePhysicalDevices,
                           VkInstance,
                           instance,
                           uint32_t *,
                           pPhysicalDeviceCount,
                           VkPhysicalDevice *,
                           pPhysicalDevices);
VK_FUNC_ERROR_CHECK_WRAPPER(vkGetPhysicalDeviceImageFormatProperties,
                           VkPhysicalDevice,
                           physicalDevice,
                           VkFormat,
                           format,
                           VkImageType,
                           type,
                           VkImageTiling,
                           tiling,
                           VkImageUsageFlags,
                           usage,
                           VkImageCreateFlags,
                           flags,
                           VkImageFormatProperties *,
                           pImageFormatProperties);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateDevice,
                           VkPhysicalDevice,
                           physicalDevice,
                           const VkDeviceCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkDevice *,
                           pDevice);
VK_FUNC_ERROR_CHECK_WRAPPER(vkEnumerateInstanceExtensionProperties,
                           const char *,
                           pLayerName,
                           uint32_t *,
                           pPropertyCount,
                           VkExtensionProperties *,
                           pProperties);
VK_FUNC_ERROR_CHECK_WRAPPER(vkEnumerateDeviceExtensionProperties,
                           VkPhysicalDevice,
                           physicalDevice,
                           const char *,
                           pLayerName,
                           uint32_t *,
                           pPropertyCount,
                           VkExtensionProperties *,
                           pProperties);
VK_FUNC_ERROR_CHECK_WRAPPER(vkEnumerateInstanceLayerProperties,
                           uint32_t *,
                           pPropertyCount,
                           VkLayerProperties *,
                           pProperties);
VK_FUNC_ERROR_CHECK_WRAPPER(vkEnumerateDeviceLayerProperties,
                           VkPhysicalDevice,
                           physicalDevice,
                           uint32_t *,
                           pPropertyCount,
                           VkLayerProperties *,
                           pProperties);
VK_FUNC_ERROR_CHECK_WRAPPER(vkQueueSubmit,
                           VkQueue,
                           queue,
                           uint32_t,
                           submitCount,
                           const VkSubmitInfo *,
                           pSubmits,
                           VkFence,
                           fence);
VK_FUNC_ERROR_CHECK_WRAPPER(vkQueueWaitIdle, VkQueue, queue);
VK_FUNC_ERROR_CHECK_WRAPPER(vkDeviceWaitIdle, VkDevice, device);
VK_FUNC_ERROR_CHECK_WRAPPER(vkAllocateMemory,
                           VkDevice,
                           device,
                           const VkMemoryAllocateInfo *,
                           pAllocateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkDeviceMemory *,
                           pMemory);
VK_FUNC_ERROR_CHECK_WRAPPER(vkMapMemory,
                           VkDevice,
                           device,
                           VkDeviceMemory,
                           memory,
                           VkDeviceSize,
                           offset,
                           VkDeviceSize,
                           size,
                           VkMemoryMapFlags,
                           flags,
                           void **,
                           ppData);
VK_FUNC_ERROR_CHECK_WRAPPER(vkFlushMappedMemoryRanges,
                           VkDevice,
                           device,
                           uint32_t,
                           memoryRangeCount,
                           const VkMappedMemoryRange *,
                           pMemoryRanges);
VK_FUNC_ERROR_CHECK_WRAPPER(vkInvalidateMappedMemoryRanges,
                           VkDevice,
                           device,
                           uint32_t,
                           memoryRangeCount,
                           const VkMappedMemoryRange *,
                           pMemoryRanges);
VK_FUNC_ERROR_CHECK_WRAPPER(vkBindBufferMemory,
                           VkDevice,
                           device,
                           VkBuffer,
                           buffer,
                           VkDeviceMemory,
                           memory,
                           VkDeviceSize,
                           memoryOffset);
VK_FUNC_ERROR_CHECK_WRAPPER(vkBindImageMemory,
                           VkDevice,
                           device,
                           VkImage,
                           image,
                           VkDeviceMemory,
                           memory,
                           VkDeviceSize,
                           memoryOffset);
VK_FUNC_ERROR_CHECK_WRAPPER(vkQueueBindSparse,
                           VkQueue,
                           queue,
                           uint32_t,
                           bindInfoCount,
                           const VkBindSparseInfo *,
                           pBindInfo,
                           VkFence,
                           fence);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateFence,
                           VkDevice,
                           device,
                           const VkFenceCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkFence *,
                           pFence);
VK_FUNC_ERROR_CHECK_WRAPPER(
    vkResetFences, VkDevice, device, uint32_t, fenceCount, const VkFence *, pFences);
VK_FUNC_ERROR_CHECK_WRAPPER(vkGetFenceStatus, VkDevice, device, VkFence, fence);
VK_FUNC_ERROR_CHECK_WRAPPER(vkWaitForFences,
                           VkDevice,
                           device,
                           uint32_t,
                           fenceCount,
                           const VkFence *,
                           pFences,
                           VkBool32,
                           waitAll,
                           uint64_t,
                           timeout);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateSemaphore,
                           VkDevice,
                           device,
                           const VkSemaphoreCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkSemaphore *,
                           pSemaphore);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateEvent,
                           VkDevice,
                           device,
                           const VkEventCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkEvent *,
                           pEvent);
VK_FUNC_ERROR_CHECK_WRAPPER(vkGetEventStatus, VkDevice, device, VkEvent, event);
VK_FUNC_ERROR_CHECK_WRAPPER(vkSetEvent, VkDevice, device, VkEvent, event);
VK_FUNC_ERROR_CHECK_WRAPPER(vkResetEvent, VkDevice, device, VkEvent, event);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateQueryPool,
                           VkDevice,
                           device,
                           const VkQueryPoolCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkQueryPool *,
                           pQueryPool);
VK_FUNC_ERROR_CHECK_WRAPPER(vkGetQueryPoolResults,
                           VkDevice,
                           device,
                           VkQueryPool,
                           queryPool,
                           uint32_t,
                           firstQuery,
                           uint32_t,
                           queryCount,
                           size_t,
                           dataSize,
                           void *,
                           pData,
                           VkDeviceSize,
                           stride,
                           VkQueryResultFlags,
                           flags);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateBuffer,
                           VkDevice,
                           device,
                           const VkBufferCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkBuffer *,
                           pBuffer);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateBufferView,
                           VkDevice,
                           device,
                           const VkBufferViewCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkBufferView *,
                           pView);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateImage,
                           VkDevice,
                           device,
                           const VkImageCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkImage *,
                           pImage);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateImageView,
                           VkDevice,
                           device,
                           const VkImageViewCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkImageView *,
                           pView);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateShaderModule,
                           VkDevice,
                           device,
                           const VkShaderModuleCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkShaderModule *,
                           pShaderModule);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreatePipelineCache,
                           VkDevice,
                           device,
                           const VkPipelineCacheCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkPipelineCache *,
                           pPipelineCache);
VK_FUNC_ERROR_CHECK_WRAPPER(vkGetPipelineCacheData,
                           VkDevice,
                           device,
                           VkPipelineCache,
                           pipelineCache,
                           size_t *,
                           pDataSize,
                           void *,
                           pData);
VK_FUNC_ERROR_CHECK_WRAPPER(vkMergePipelineCaches,
                           VkDevice,
                           device,
                           VkPipelineCache,
                           dstCache,
                           uint32_t,
                           srcCacheCount,
                           const VkPipelineCache *,
                           pSrcCaches);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateGraphicsPipelines,
                           VkDevice,
                           device,
                           VkPipelineCache,
                           pipelineCache,
                           uint32_t,
                           createInfoCount,
                           const VkGraphicsPipelineCreateInfo *,
                           pCreateInfos,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkPipeline *,
                           pPipelines);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateComputePipelines,
                           VkDevice,
                           device,
                           VkPipelineCache,
                           pipelineCache,
                           uint32_t,
                           createInfoCount,
                           const VkComputePipelineCreateInfo *,
                           pCreateInfos,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkPipeline *,
                           pPipelines);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreatePipelineLayout,
                           VkDevice,
                           device,
                           const VkPipelineLayoutCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkPipelineLayout *,
                           pPipelineLayout);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateSampler,
                           VkDevice,
                           device,
                           const VkSamplerCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkSampler *,
                           pSampler);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateDescriptorSetLayout,
                           VkDevice,
                           device,
                           const VkDescriptorSetLayoutCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkDescriptorSetLayout *,
                           pSetLayout);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateDescriptorPool,
                           VkDevice,
                           device,
                           const VkDescriptorPoolCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkDescriptorPool *,
                           pDescriptorPool);
VK_FUNC_ERROR_CHECK_WRAPPER(vkResetDescriptorPool,
                           VkDevice,
                           device,
                           VkDescriptorPool,
                           descriptorPool,
                           VkDescriptorPoolResetFlags,
                           flags);
VK_FUNC_ERROR_CHECK_WRAPPER(vkAllocateDescriptorSets,
                           VkDevice,
                           device,
                           const VkDescriptorSetAllocateInfo *,
                           pAllocateInfo,
                           VkDescriptorSet *,
                           pDescriptorSets);
VK_FUNC_ERROR_CHECK_WRAPPER(vkFreeDescriptorSets,
                           VkDevice,
                           device,
                           VkDescriptorPool,
                           descriptorPool,
                           uint32_t,
                           descriptorSetCount,
                           const VkDescriptorSet *,
                           pDescriptorSets);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateFramebuffer,
                           VkDevice,
                           device,
                           const VkFramebufferCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkFramebuffer *,
                           pFramebuffer);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateRenderPass,
                           VkDevice,
                           device,
                           const VkRenderPassCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkRenderPass *,
                           pRenderPass);
VK_FUNC_ERROR_CHECK_WRAPPER(vkCreateCommandPool,
                           VkDevice,
                           device,
                           const VkCommandPoolCreateInfo *,
                           pCreateInfo,
                           const VkAllocationCallbacks *,
                           pAllocator,
                           VkCommandPool *,
                           pCommandPool);
VK_FUNC_ERROR_CHECK_WRAPPER(vkResetCommandPool,
                           VkDevice,
                           device,
                           VkCommandPool,
                           commandPool,
                           VkCommandPoolResetFlags,
                           flags);
VK_FUNC_ERROR_CHECK_WRAPPER(vkAllocateCommandBuffers,
                           VkDevice,
                           device,
                           const VkCommandBufferAllocateInfo *,
                           pAllocateInfo,
                           VkCommandBuffer *,
                           pCommandBuffers);
VK_FUNC_ERROR_CHECK_WRAPPER(vkBeginCommandBuffer,
                           VkCommandBuffer,
                           commandBuffer,
                           const VkCommandBufferBeginInfo *,
                           pBeginInfo);
VK_FUNC_ERROR_CHECK_WRAPPER(vkEndCommandBuffer, VkCommandBuffer, commandBuffer);
VK_FUNC_ERROR_CHECK_WRAPPER(
    vkResetCommandBuffer, VkCommandBuffer, commandBuffer, VkCommandBufferResetFlags, flags);

VK_FUNC_VOID_WRAPPER(vkDestroyInstance,
                     VkInstance, instance,
                     const VkAllocationCallbacks *,pAllocator)

VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceFeatures,VkPhysicalDevice, physicalDevice,
                                                         VkPhysicalDeviceFeatures *,pFeatures);
VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceFormatProperties,
    VkPhysicalDevice ,physicalDevice, VkFormat ,format, VkFormatProperties *,pFormatProperties);

VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceProperties,
    VkPhysicalDevice ,physicalDevice, VkPhysicalDeviceProperties *,pProperties);
VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceQueueFamilyProperties,
    VkPhysicalDevice ,physicalDevice,
    uint32_t *,pQueueFamilyPropertyCount,
    VkQueueFamilyProperties *,pQueueFamilyProperties);
VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceMemoryProperties,
    VkPhysicalDevice ,physicalDevice, VkPhysicalDeviceMemoryProperties *,pMemoryProperties);

typedef PFN_vkVoidFunction(VKAPI_PTR *PFN_vkGetDeviceProcAddr)(VkDevice device, const char *pName);

VK_FUNC_VOID_WRAPPER(vkDestroyDevice,VkDevice, device,
                                             const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_WRAPPER(vkGetDeviceQueue,VkDevice ,device,
                                              uint32_t ,queueFamilyIndex,
                                              uint32_t ,queueIndex,
                                              VkQueue *,pQueue);

VK_FUNC_VOID_WRAPPER(vkFreeMemory,VkDevice, device,
                                          VkDeviceMemory, memory,
                                          const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkUnmapMemory,VkDevice ,device, VkDeviceMemory ,memory);
VK_FUNC_VOID_WRAPPER(vkGetDeviceMemoryCommitment,VkDevice, device,
                                                         VkDeviceMemory ,memory,
                                                         VkDeviceSize *,pCommittedMemoryInBytes);
VK_FUNC_VOID_WRAPPER(vkGetBufferMemoryRequirements,
    VkDevice ,device, VkBuffer ,buffer, VkMemoryRequirements *,pMemoryRequirements);
VK_FUNC_VOID_WRAPPER(vkGetImageMemoryRequirements,
    VkDevice ,device, VkImage, image, VkMemoryRequirements *,pMemoryRequirements);
VK_FUNC_VOID_WRAPPER(vkGetImageSparseMemoryRequirements,
    VkDevice ,device,
    VkImage ,image,
    uint32_t *,pSparseMemoryRequirementCount,
    VkSparseImageMemoryRequirements *,pSparseMemoryRequirements);
VK_FUNC_VOID_WRAPPER(vkGetPhysicalDeviceSparseImageFormatProperties,
    VkPhysicalDevice ,physicalDevice,
    VkFormat, format,
    VkImageType ,type,
    VkSampleCountFlagBits, samples,
    VkImageUsageFlags, usage,
    VkImageTiling ,tiling,
    uint32_t *,pPropertyCount,
    VkSparseImageFormatProperties *,pProperties);

VK_FUNC_VOID_WRAPPER(vkDestroyFence,VkDevice, device,
                                            VkFence, fence,
                                            const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroySemaphore,VkDevice, device,
                                                VkSemaphore, semaphore,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyEvent,VkDevice ,device,
                                            VkEvent ,event,
                                            const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyQueryPool,VkDevice ,device,
                                                VkQueryPool ,queryPool,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyBuffer,VkDevice ,device,
                                             VkBuffer ,buffer,
                                             const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyBufferView,VkDevice, device,
                                                 VkBufferView ,bufferView,
                                                 const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyImage,VkDevice ,device,
                                            VkImage, image,
                                            const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_WRAPPER(vkGetImageSubresourceLayout,VkDevice ,device,
                                                         VkImage ,image,
                                                         const VkImageSubresource *,pSubresource,
                                                         VkSubresourceLayout *,pLayout);

VK_FUNC_VOID_WRAPPER(vkDestroyImageView,VkDevice ,device,
                                                VkImageView, imageView,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyShaderModule,VkDevice ,device,
                                                   VkShaderModule ,shaderModule,
                                                   const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyPipelineCache,VkDevice, device,
                                                    VkPipelineCache ,pipelineCache,
                                                    const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyPipeline,VkDevice ,device,
                                               VkPipeline ,pipeline,
                                               const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyPipelineLayout,VkDevice ,device,
                                                     VkPipelineLayout ,pipelineLayout,
                                                     const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroySampler,VkDevice ,device,
                                              VkSampler ,sampler,
                                              const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyDescriptorSetLayout,
    VkDevice ,device,
    VkDescriptorSetLayout, descriptorSetLayout,
    const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyDescriptorPool,VkDevice ,device,
                                                     VkDescriptorPool ,descriptorPool,
                                                     const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_WRAPPER(vkUpdateDescriptorSets,VkDevice ,device,
                                                    uint32_t ,descriptorWriteCount,
                                                    const VkWriteDescriptorSet *,pDescriptorWrites,
                                                    uint32_t ,descriptorCopyCount,
                                                    const VkCopyDescriptorSet *,pDescriptorCopies);

VK_FUNC_VOID_WRAPPER(vkDestroyFramebuffer,VkDevice ,device,
                                                  VkFramebuffer ,framebuffer,
                                                  const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_WRAPPER(vkDestroyRenderPass,VkDevice ,device,
                                                 VkRenderPass ,renderPass,
                                                 const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_WRAPPER(vkGetRenderAreaGranularity,VkDevice, device,
                                                        VkRenderPass, renderPass,
                                                        VkExtent2D *,pGranularity);

VK_FUNC_VOID_WRAPPER(vkDestroyCommandPool,VkDevice ,device,
                                                  VkCommandPool ,commandPool,
                                                  const VkAllocationCallbacks *,pAllocator);


VK_FUNC_VOID_WRAPPER(vkFreeCommandBuffers,VkDevice, device,
                                                  VkCommandPool, commandPool,
                                                  uint32_t ,commandBufferCount,
                                                  const VkCommandBuffer *,pCommandBuffers);



VK_FUNC_VOID_WRAPPER(vkCmdBindPipeline,VkCommandBuffer, commandBuffer,
                                               VkPipelineBindPoint ,pipelineBindPoint,
                                               VkPipeline ,pipeline);
VK_FUNC_VOID_WRAPPER(vkCmdSetViewport,VkCommandBuffer, commandBuffer,
                                              uint32_t ,firstViewport,
                                              uint32_t ,viewportCount,
                                              const VkViewport *,pViewports);
VK_FUNC_VOID_WRAPPER(vkCmdSetScissor,VkCommandBuffer ,commandBuffer,
                                             uint32_t ,firstScissor,
                                             uint32_t ,scissorCount,
                                             const VkRect2D *,pScissors);
VK_FUNC_VOID_WRAPPER(vkCmdSetLineWidth,VkCommandBuffer ,commandBuffer, float ,lineWidth);
VK_FUNC_VOID_WRAPPER(vkCmdSetDepthBias,VkCommandBuffer, commandBuffer,
                                               float ,depthBiasConstantFactor,
                                               float ,depthBiasClamp,
                                               float ,depthBiasSlopeFactor);
VK_FUNC_VOID_WRAPPER(vkCmdSetBlendConstants,VkCommandBuffer, commandBuffer, const float*, blendConstants);

VK_FUNC_VOID_WRAPPER(vkCmdSetDepthBounds,VkCommandBuffer ,commandBuffer,
                                                 float ,minDepthBounds,
                                                 float, maxDepthBounds);
VK_FUNC_VOID_WRAPPER(vkCmdSetStencilCompareMask,VkCommandBuffer ,commandBuffer,
                                                        VkStencilFaceFlags ,faceMask,
                                                        uint32_t ,compareMask);
VK_FUNC_VOID_WRAPPER(vkCmdSetStencilWriteMask,VkCommandBuffer, commandBuffer,
                                                      VkStencilFaceFlags ,faceMask,
                                                      uint32_t, writeMask);
VK_FUNC_VOID_WRAPPER(vkCmdSetStencilReference,VkCommandBuffer ,commandBuffer,
                                                      VkStencilFaceFlags ,faceMask,
                                                      uint32_t, reference);
VK_FUNC_VOID_WRAPPER(vkCmdBindDescriptorSets,VkCommandBuffer ,commandBuffer,
                                                     VkPipelineBindPoint ,pipelineBindPoint,
                                                     VkPipelineLayout, layout,
                                                     uint32_t ,firstSet,
                                                     uint32_t ,descriptorSetCount,
                                                     const VkDescriptorSet *,pDescriptorSets,
                                                     uint32_t ,dynamicOffsetCount,
                                                     const uint32_t *,pDynamicOffsets);
VK_FUNC_VOID_WRAPPER(vkCmdBindIndexBuffer,VkCommandBuffer ,commandBuffer,
                                                  VkBuffer ,buffer,
                                                  VkDeviceSize ,offset,
                                                  VkIndexType ,indexType);
VK_FUNC_VOID_WRAPPER(vkCmdBindVertexBuffers,VkCommandBuffer, commandBuffer,
                                                    uint32_t ,firstBinding,
                                                    uint32_t ,bindingCount,
                                                    const VkBuffer *,pBuffers,
                                                    const VkDeviceSize *,pOffsets);
VK_FUNC_VOID_WRAPPER(vkCmdDraw,VkCommandBuffer, commandBuffer,
                                       uint32_t ,vertexCount,
                                       uint32_t ,instanceCount,
                                       uint32_t ,firstVertex,
                                       uint32_t ,firstInstance);
VK_FUNC_VOID_WRAPPER(vkCmdDrawIndexed,VkCommandBuffer ,commandBuffer,
                                              uint32_t, indexCount,
                                              uint32_t ,instanceCount,
                                              uint32_t ,firstIndex,
                                              int32_t ,vertexOffset,
                                              uint32_t, firstInstance);
VK_FUNC_VOID_WRAPPER(vkCmdDrawIndirect,VkCommandBuffer ,commandBuffer,
                                               VkBuffer ,buffer,
                                               VkDeviceSize ,offset,
                                               uint32_t ,drawCount,
                                               uint32_t ,stride);
VK_FUNC_VOID_WRAPPER(vkCmdDrawIndexedIndirect,VkCommandBuffer ,commandBuffer,
                                                      VkBuffer ,buffer,
                                                      VkDeviceSize, offset,
                                                      uint32_t ,drawCount,
                                                      uint32_t ,stride);
VK_FUNC_VOID_WRAPPER(vkCmdDispatch,VkCommandBuffer, commandBuffer,
                                           uint32_t ,groupCountX,
                                           uint32_t ,groupCountY,
                                           uint32_t ,groupCountZ);
VK_FUNC_VOID_WRAPPER(vkCmdDispatchIndirect,VkCommandBuffer ,commandBuffer,
                                                   VkBuffer ,buffer,
                                                   VkDeviceSize ,offset);
VK_FUNC_VOID_WRAPPER(vkCmdCopyBuffer,VkCommandBuffer, commandBuffer,
                                             VkBuffer ,srcBuffer,
                                             VkBuffer ,dstBuffer,
                                             uint32_t ,regionCount,
                                             const VkBufferCopy *,pRegions);
VK_FUNC_VOID_WRAPPER(vkCmdCopyImage,VkCommandBuffer ,commandBuffer,
                                            VkImage ,srcImage,
                                            VkImageLayout ,srcImageLayout,
                                            VkImage ,dstImage,
                                            VkImageLayout, dstImageLayout,
                                            uint32_t ,regionCount,
                                            const VkImageCopy *,pRegions);
VK_FUNC_VOID_WRAPPER(vkCmdBlitImage,VkCommandBuffer ,commandBuffer,
                                            VkImage ,srcImage,
                                            VkImageLayout ,srcImageLayout,
                                            VkImage ,dstImage,
                                            VkImageLayout ,dstImageLayout,
                                            uint32_t ,regionCount,
                                            const VkImageBlit *,pRegions,
                                            VkFilter ,filter);
VK_FUNC_VOID_WRAPPER(vkCmdCopyBufferToImage,VkCommandBuffer, commandBuffer,
                                                    VkBuffer ,srcBuffer,
                                                    VkImage ,dstImage,
                                                    VkImageLayout ,dstImageLayout,
                                                    uint32_t ,regionCount,
                                                    const VkBufferImageCopy *,pRegions);
VK_FUNC_VOID_WRAPPER(vkCmdCopyImageToBuffer,VkCommandBuffer, commandBuffer,
                                                    VkImage ,srcImage,
                                                    VkImageLayout ,srcImageLayout,
                                                    VkBuffer ,dstBuffer,
                                                    uint32_t ,regionCount,
                                                    const VkBufferImageCopy *,pRegions);
VK_FUNC_VOID_WRAPPER(vkCmdUpdateBuffer,VkCommandBuffer, commandBuffer,
                                               VkBuffer, dstBuffer,
                                               VkDeviceSize ,dstOffset,
                                               VkDeviceSize ,dataSize,
                                               const void *,pData);
VK_FUNC_VOID_WRAPPER(vkCmdFillBuffer,VkCommandBuffer ,commandBuffer,
                                             VkBuffer ,dstBuffer,
                                             VkDeviceSize ,dstOffset,
                                             VkDeviceSize ,size,
                                             uint32_t ,data);

VK_FUNC_VOID_WRAPPER(vkCmdClearColorImage,VkCommandBuffer, commandBuffer,
                                                  VkImage ,image,
                                                  VkImageLayout ,imageLayout,
                                                  const VkClearColorValue *,pColor,
                                                  uint32_t ,rangeCount,
                                                  const VkImageSubresourceRange *,pRanges);

VK_FUNC_VOID_WRAPPER(vkCmdClearDepthStencilImage,
    VkCommandBuffer ,commandBuffer,
    VkImage ,image,
    VkImageLayout, imageLayout,
    const VkClearDepthStencilValue *,pDepthStencil,
    uint32_t ,rangeCount,
    const VkImageSubresourceRange *,pRanges);
VK_FUNC_VOID_WRAPPER(vkCmdClearAttachments,VkCommandBuffer ,commandBuffer,
                                                   uint32_t, attachmentCount,
                                                   const VkClearAttachment *,pAttachments,
                                                   uint32_t, rectCount,
                                                   const VkClearRect *,pRects);
VK_FUNC_VOID_WRAPPER(vkCmdResolveImage,VkCommandBuffer, commandBuffer,
                                               VkImage ,srcImage,
                                               VkImageLayout ,srcImageLayout,
                                               VkImage ,dstImage,
                                               VkImageLayout ,dstImageLayout,
                                               uint32_t ,regionCount,
                                               const VkImageResolve *,pRegions);
VK_FUNC_VOID_WRAPPER(vkCmdSetEvent,VkCommandBuffer ,commandBuffer,
                                           VkEvent ,event,
                                           VkPipelineStageFlags ,stageMask);
VK_FUNC_VOID_WRAPPER(vkCmdResetEvent,VkCommandBuffer ,commandBuffer,
                                             VkEvent ,event,
                                             VkPipelineStageFlags ,stageMask);
VK_FUNC_VOID_WRAPPER(vkCmdWaitEvents,VkCommandBuffer ,commandBuffer,
                                             uint32_t ,eventCount,
                                             const VkEvent *,pEvents,
                                             VkPipelineStageFlags ,srcStageMask,
                                             VkPipelineStageFlags ,dstStageMask,
                                             uint32_t ,memoryBarrierCount,
                                             const VkMemoryBarrier *,pMemoryBarriers,
                                             uint32_t ,bufferMemoryBarrierCount,
                                             const VkBufferMemoryBarrier *,pBufferMemoryBarriers,
                                             uint32_t ,imageMemoryBarrierCount,
                                             const VkImageMemoryBarrier *,pImageMemoryBarriers);
VK_FUNC_VOID_WRAPPER(vkCmdPipelineBarrier,
    VkCommandBuffer ,commandBuffer,
    VkPipelineStageFlags ,srcStageMask,
    VkPipelineStageFlags ,dstStageMask,
    VkDependencyFlags ,dependencyFlags,
    uint32_t ,memoryBarrierCount,
    const VkMemoryBarrier *,pMemoryBarriers,
    uint32_t ,bufferMemoryBarrierCount,
    const VkBufferMemoryBarrier *,pBufferMemoryBarriers,
    uint32_t ,imageMemoryBarrierCount,
    const VkImageMemoryBarrier *,pImageMemoryBarriers);
VK_FUNC_VOID_WRAPPER(vkCmdBeginQuery,VkCommandBuffer, commandBuffer,
                                             VkQueryPool ,queryPool,
                                             uint32_t ,query,
                                             VkQueryControlFlags ,flags);
VK_FUNC_VOID_WRAPPER(vkCmdEndQuery,VkCommandBuffer, commandBuffer,
                                           VkQueryPool ,queryPool,
                                           uint32_t ,query);
VK_FUNC_VOID_WRAPPER(vkCmdResetQueryPool,VkCommandBuffer, commandBuffer,
                                                 VkQueryPool ,queryPool,
                                                 uint32_t ,firstQuery,
                                                 uint32_t ,queryCount);
VK_FUNC_VOID_WRAPPER(vkCmdWriteTimestamp,VkCommandBuffer ,commandBuffer,
                                                 VkPipelineStageFlagBits ,pipelineStage,
                                                 VkQueryPool ,queryPool,
                                                 uint32_t ,query);
VK_FUNC_VOID_WRAPPER(vkCmdCopyQueryPoolResults,VkCommandBuffer, commandBuffer,
                                                       VkQueryPool, queryPool,
                                                       uint32_t ,firstQuery,
                                                       uint32_t ,queryCount,
                                                       VkBuffer ,dstBuffer,
                                                       VkDeviceSize ,dstOffset,
                                                       VkDeviceSize ,stride,
                                                       VkQueryResultFlags ,flags);
VK_FUNC_VOID_WRAPPER(vkCmdPushConstants,VkCommandBuffer ,commandBuffer,
                                                VkPipelineLayout ,layout,
                                                VkShaderStageFlags, stageFlags,
                                                uint32_t ,offset,
                                                uint32_t ,size,
                                                const void *,pValues);
VK_FUNC_VOID_WRAPPER(vkCmdBeginRenderPass,VkCommandBuffer ,commandBuffer,
                                                  const VkRenderPassBeginInfo *,pRenderPassBegin,
                                                  VkSubpassContents ,contents);
VK_FUNC_VOID_WRAPPER(vkCmdNextSubpass,VkCommandBuffer, commandBuffer,
                                              VkSubpassContents ,contents);
VK_FUNC_VOID_WRAPPER(vkCmdEndRenderPass,VkCommandBuffer, commandBuffer);
VK_FUNC_VOID_WRAPPER(vkCmdExecuteCommands,VkCommandBuffer, commandBuffer,
                                                  uint32_t ,commandBufferCount,
                                                  const VkCommandBuffer *,pCommandBuffers);


namespace blender {
namespace gpu {
namespace debug {


static VkResult DynamicLibInitialize(void)
{
#if defined(_WIN32)
  HMODULE module = LoadLibraryA("vulkan-1.dll");
  if (!module) {
    return VK_ERROR_INITIALIZATION_FAILED;
  }
#elif defined(__APPLE__)
  void *module = dlopen("libvulkan.dylib", RTLD_NOW | RTLD_LOCAL);
  if (!module) {
    module = dlopen("libvulkan.1.dylib", RTLD_NOW | RTLD_LOCAL);
  }
  if (!module) {
    module = dlopen("libMoltenVK.dylib", RTLD_NOW | RTLD_LOCAL);
  }
  if (!module) {
    return VK_ERROR_INITIALIZATION_FAILED;
  }

  vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym(module, "vkGetInstanceProcAddr");
#else
  void *module = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
  if (!module) {
    module = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
  }
  if (!module) {
    return VK_ERROR_INITIALIZATION_FAILED;
  }

  vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym(module, "vkGetInstanceProcAddr");
#endif

  // note: function pointer is cast through void function pointer to silence cast-function-type
  // warning on gcc8
  
  tools.vk_wrapper.vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)GetFuncAddress(
      module, "vkGetInstanceProcAddr");
  tools.vk_wrapper.pfvkCreateInstance = (PFN_vkCreateInstance)GetFuncAddress(
      module, "vkCreateInstance");
  tools.vk_wrapper.vkDestroyInstance = (PFN_vkDestroyInstance)GetFuncAddress(
      module, "vkDestroyInstance");
  tools.vk_wrapper.pfvkEnumeratePhysicalDevices = (PFN_vkEnumeratePhysicalDevices)GetFuncAddress(
      module, "vkEnumeratePhysicalDevices");
  tools.vk_wrapper.vkGetPhysicalDeviceFeatures = (PFN_vkGetPhysicalDeviceFeatures)GetFuncAddress(
      module, "vkGetPhysicalDeviceFeatures");
  tools.vk_wrapper.pfvkGetPhysicalDeviceImageFormatProperties = (PFN_vkGetPhysicalDeviceImageFormatProperties)GetFuncAddress(module, "vkGetPhysicalDeviceImageFormatProperties");

  tools.vk_wrapper.vkGetPhysicalDeviceFormatProperties = (PFN_vkGetPhysicalDeviceFormatProperties)GetFuncAddress(module, "vkGetPhysicalDeviceFormatProperties");

  tools.vk_wrapper.vkGetPhysicalDeviceSparseImageFormatProperties =
      (PFN_vkGetPhysicalDeviceSparseImageFormatProperties)GetFuncAddress(module, "vkGetPhysicalDeviceSparseImageFormatProperties");

  tools.vk_wrapper.vkGetPhysicalDeviceProperties = (PFN_vkGetPhysicalDeviceProperties)GetFuncAddress(module, "vkGetPhysicalDeviceProperties");
    
  tools.vk_wrapper.vkGetPhysicalDeviceQueueFamilyProperties =(PFN_vkGetPhysicalDeviceQueueFamilyProperties)GetFuncAddress(module, "vkGetPhysicalDeviceQueueFamilyProperties");

  tools.vk_wrapper.vkGetPhysicalDeviceMemoryProperties = (PFN_vkGetPhysicalDeviceMemoryProperties)GetFuncAddress(module, "vkGetPhysicalDeviceMemoryProperties");

  tools.vk_wrapper.pfvkEnumerateInstanceExtensionProperties = (PFN_vkEnumerateInstanceExtensionProperties)GetFuncAddress(module, "vkEnumerateInstanceExtensionProperties");

  tools.vk_wrapper.pfvkEnumerateInstanceLayerProperties = (PFN_vkEnumerateInstanceLayerProperties)GetFuncAddress(module, "vkEnumerateInstanceLayerProperties");

  return VK_SUCCESS;
}


/**
 * A function that loads a function for an instance of Vulkan.
 * Each extension must be enabled in the instance to be loaded here.
 * DLL load seems unnecessary for now.
 */

static void BlenderVulkanLoadInstance(VkInstance& context)
{
#define load tools.vk_wrapper.vkGetInstanceProcAddr


  tools.vk_wrapper.vkGetDeviceProcAddr = (PFN_vkGetDeviceProcAddr)load(context, "vkGetDeviceProcAddr");

  tools.vk_wrapper.pfvkEnumerateDeviceExtensionProperties = (PFN_vkEnumerateDeviceExtensionProperties)load(
      context, "vkEnumerateDeviceExtensionProperties");
  tools.vk_wrapper.pfvkEnumerateDeviceLayerProperties = (PFN_vkEnumerateDeviceLayerProperties)load(
      context, "vkEnumerateDeviceLayerProperties");


#if defined(VK_EXT_debug_utils)
  pfvkCmdBeginDebugUtilsLabelEXT = (PFN_vkCmdBeginDebugUtilsLabelEXT)load(
      context, "vkCmdBeginDebugUtilsLabelEXT");
  pfvkCmdEndDebugUtilsLabelEXT = (PFN_vkCmdEndDebugUtilsLabelEXT)load(
      context, "vkCmdEndDebugUtilsLabelEXT");
  pfvkCmdInsertDebugUtilsLabelEXT = (PFN_vkCmdInsertDebugUtilsLabelEXT)load(
      context, "vkCmdInsertDebugUtilsLabelEXT");
  pfvkCreateDebugUtilsMessengerEXT = (PFN_vkCreateDebugUtilsMessengerEXT)load(
      context, "vkCreateDebugUtilsMessengerEXT");
  pfvkDestroyDebugUtilsMessengerEXT = (PFN_vkDestroyDebugUtilsMessengerEXT)load(
      context, "vkDestroyDebugUtilsMessengerEXT");
  pfvkQueueBeginDebugUtilsLabelEXT = (PFN_vkQueueBeginDebugUtilsLabelEXT)load(
      context, "vkQueueBeginDebugUtilsLabelEXT");
  pfvkQueueEndDebugUtilsLabelEXT = (PFN_vkQueueEndDebugUtilsLabelEXT)load(
      context, "vkQueueEndDebugUtilsLabelEXT");
  pfvkQueueInsertDebugUtilsLabelEXT = (PFN_vkQueueInsertDebugUtilsLabelEXT)load(
      context, "vkQueueInsertDebugUtilsLabelEXT");
  pfvkSetDebugUtilsObjectNameEXT = (PFN_vkSetDebugUtilsObjectNameEXT)load(
      context, "vkSetDebugUtilsObjectNameEXT");
  pfvkSetDebugUtilsObjectTagEXT = (PFN_vkSetDebugUtilsObjectTagEXT)load(
      context, "vkSetDebugUtilsObjectTagEXT");
  pfvkSubmitDebugUtilsMessageEXT = (PFN_vkSubmitDebugUtilsMessageEXT)load(
      context, "vkSubmitDebugUtilsMessageEXT");

  if (pfvkCmdBeginDebugUtilsLabelEXT != nullptr) {
    tools.enabled = true;
  }

#endif /* defined(VK_EXT_debug_utils) */

#undef load
}

void vulkanLoadDevice(VkDevice& device)
{

#define load tools.vk_wrapper.vkGetDeviceProcAddr

  tools.vk_wrapper.pfvkAllocateCommandBuffers = (PFN_vkAllocateCommandBuffers)load(device,
                                                                  "vkAllocateCommandBuffers");
  tools.vk_wrapper.pfvkAllocateDescriptorSets = (PFN_vkAllocateDescriptorSets)load(device,
                                                                  "vkAllocateDescriptorSets");
  tools.vk_wrapper.pfvkAllocateMemory = (PFN_vkAllocateMemory)load(device, "vkAllocateMemory");
  tools.vk_wrapper.pfvkBeginCommandBuffer = (PFN_vkBeginCommandBuffer)load(device, "vkBeginCommandBuffer");
  tools.vk_wrapper.pfvkBindBufferMemory = (PFN_vkBindBufferMemory)load(device, "vkBindBufferMemory");
  tools.vk_wrapper.pfvkBindImageMemory = (PFN_vkBindImageMemory)load(device, "vkBindImageMemory");
  tools.vk_wrapper.vkCmdBeginQuery = (PFN_vkCmdBeginQuery)load(device, "vkCmdBeginQuery");
  tools.vk_wrapper.vkCmdBeginRenderPass = (PFN_vkCmdBeginRenderPass)load(device, "vkCmdBeginRenderPass");
  tools.vk_wrapper.vkCmdBindDescriptorSets = (PFN_vkCmdBindDescriptorSets)load(device, "vkCmdBindDescriptorSets");
  tools.vk_wrapper.vkCmdBindIndexBuffer = (PFN_vkCmdBindIndexBuffer)load(device, "vkCmdBindIndexBuffer");
  tools.vk_wrapper.vkCmdBindPipeline = (PFN_vkCmdBindPipeline)load(device, "vkCmdBindPipeline");
  tools.vk_wrapper.vkCmdBindVertexBuffers = (PFN_vkCmdBindVertexBuffers)load(device, "vkCmdBindVertexBuffers");
  tools.vk_wrapper.vkCmdBlitImage = (PFN_vkCmdBlitImage)load(device, "vkCmdBlitImage");
  tools.vk_wrapper.vkCmdClearAttachments = (PFN_vkCmdClearAttachments)load(device, "vkCmdClearAttachments");
  tools.vk_wrapper.vkCmdClearColorImage = (PFN_vkCmdClearColorImage)load(device, "vkCmdClearColorImage");
  tools.vk_wrapper.vkCmdClearDepthStencilImage = (PFN_vkCmdClearDepthStencilImage)load(
      device, "vkCmdClearDepthStencilImage");
  tools.vk_wrapper.vkCmdCopyBuffer = (PFN_vkCmdCopyBuffer)load(device, "vkCmdCopyBuffer");
  tools.vk_wrapper.vkCmdCopyBufferToImage = (PFN_vkCmdCopyBufferToImage)load(device, "vkCmdCopyBufferToImage");
  tools.vk_wrapper.vkCmdCopyImage = (PFN_vkCmdCopyImage)load(device, "vkCmdCopyImage");
  tools.vk_wrapper.vkCmdCopyImageToBuffer = (PFN_vkCmdCopyImageToBuffer)load(device, "vkCmdCopyImageToBuffer");
  tools.vk_wrapper.vkCmdCopyQueryPoolResults = (PFN_vkCmdCopyQueryPoolResults)load(device,
                                                                  "vkCmdCopyQueryPoolResults");
  tools.vk_wrapper.vkCmdDispatch = (PFN_vkCmdDispatch)load(device, "vkCmdDispatch");
  tools.vk_wrapper.vkCmdDispatchIndirect = (PFN_vkCmdDispatchIndirect)load(device, "vkCmdDispatchIndirect");
  tools.vk_wrapper.vkCmdDraw = (PFN_vkCmdDraw)load(device, "vkCmdDraw");
  tools.vk_wrapper.vkCmdDrawIndexed = (PFN_vkCmdDrawIndexed)load(device, "vkCmdDrawIndexed");
  tools.vk_wrapper.vkCmdDrawIndexedIndirect = (PFN_vkCmdDrawIndexedIndirect)load(device,
                                                                "vkCmdDrawIndexedIndirect");
  tools.vk_wrapper.vkCmdDrawIndirect = (PFN_vkCmdDrawIndirect)load(device, "vkCmdDrawIndirect");
  tools.vk_wrapper.vkCmdEndQuery = (PFN_vkCmdEndQuery)load(device, "vkCmdEndQuery");
  tools.vk_wrapper.vkCmdEndRenderPass = (PFN_vkCmdEndRenderPass)load(device, "vkCmdEndRenderPass");
  tools.vk_wrapper.vkCmdExecuteCommands = (PFN_vkCmdExecuteCommands)load(device, "vkCmdExecuteCommands");
  tools.vk_wrapper.vkCmdFillBuffer = (PFN_vkCmdFillBuffer)load(device, "vkCmdFillBuffer");
  tools.vk_wrapper.vkCmdNextSubpass = (PFN_vkCmdNextSubpass)load(device, "vkCmdNextSubpass");
  tools.vk_wrapper.vkCmdPipelineBarrier = (PFN_vkCmdPipelineBarrier)load(device, "vkCmdPipelineBarrier");
  tools.vk_wrapper.vkCmdPushConstants = (PFN_vkCmdPushConstants)load(device, "vkCmdPushConstants");
  tools.vk_wrapper.vkCmdResetEvent = (PFN_vkCmdResetEvent)load(device, "vkCmdResetEvent");
  tools.vk_wrapper.vkCmdResetQueryPool = (PFN_vkCmdResetQueryPool)load(device, "vkCmdResetQueryPool");
  tools.vk_wrapper.vkCmdResolveImage = (PFN_vkCmdResolveImage)load(device, "vkCmdResolveImage");
  tools.vk_wrapper.vkCmdSetBlendConstants = (PFN_vkCmdSetBlendConstants)load(device, "vkCmdSetBlendConstants");
  tools.vk_wrapper.vkCmdSetDepthBias = (PFN_vkCmdSetDepthBias)load(device, "vkCmdSetDepthBias");
  tools.vk_wrapper.vkCmdSetDepthBounds = (PFN_vkCmdSetDepthBounds)load(device, "vkCmdSetDepthBounds");
  tools.vk_wrapper.vkCmdSetEvent = (PFN_vkCmdSetEvent)load(device, "vkCmdSetEvent");
  tools.vk_wrapper.vkCmdSetLineWidth = (PFN_vkCmdSetLineWidth)load(device, "vkCmdSetLineWidth");
  tools.vk_wrapper.vkCmdSetScissor = (PFN_vkCmdSetScissor)load(device, "vkCmdSetScissor");
  tools.vk_wrapper.vkCmdSetStencilCompareMask = (PFN_vkCmdSetStencilCompareMask)load(device,
                                                                    "vkCmdSetStencilCompareMask");
  tools.vk_wrapper.vkCmdSetStencilReference = (PFN_vkCmdSetStencilReference)load(device,
                                                                "vkCmdSetStencilReference");
  tools.vk_wrapper.vkCmdSetStencilWriteMask = (PFN_vkCmdSetStencilWriteMask)load(device,
                                                                "vkCmdSetStencilWriteMask");
  tools.vk_wrapper.vkCmdSetViewport = (PFN_vkCmdSetViewport)load(device, "vkCmdSetViewport");
  tools.vk_wrapper.vkCmdUpdateBuffer = (PFN_vkCmdUpdateBuffer)load(device, "vkCmdUpdateBuffer");
  tools.vk_wrapper.vkCmdWaitEvents = (PFN_vkCmdWaitEvents)load(device, "vkCmdWaitEvents");
  tools.vk_wrapper.vkCmdWriteTimestamp = (PFN_vkCmdWriteTimestamp)load(device, "vkCmdWriteTimestamp");
  tools.vk_wrapper.pfvkCreateBuffer = (PFN_vkCreateBuffer)load(device, "vkCreateBuffer");
  tools.vk_wrapper.pfvkCreateBufferView = (PFN_vkCreateBufferView)load(device, "vkCreateBufferView");
  tools.vk_wrapper.pfvkCreateCommandPool = (PFN_vkCreateCommandPool)load(device, "vkCreateCommandPool");
  tools.vk_wrapper.pfvkCreateComputePipelines = (PFN_vkCreateComputePipelines)load(device,
                                                                  "vkCreateComputePipelines");
  tools.vk_wrapper.pfvkCreateDescriptorPool = (PFN_vkCreateDescriptorPool)load(device, "vkCreateDescriptorPool");
  tools.vk_wrapper.pfvkCreateDescriptorSetLayout = (PFN_vkCreateDescriptorSetLayout)load(
      device, "vkCreateDescriptorSetLayout");
  tools.vk_wrapper.pfvkCreateDevice = (PFN_vkCreateDevice)load(device, "vkCreateDevice");
  tools.vk_wrapper.pfvkCreateEvent = (PFN_vkCreateEvent)load(device, "vkCreateEvent");
  tools.vk_wrapper.pfvkCreateFence = (PFN_vkCreateFence)load(device, "vkCreateFence");
  tools.vk_wrapper.pfvkCreateFramebuffer = (PFN_vkCreateFramebuffer)load(device, "vkCreateFramebuffer");
  tools.vk_wrapper.pfvkCreateGraphicsPipelines = (PFN_vkCreateGraphicsPipelines)load(device,
                                                                    "vkCreateGraphicsPipelines");
  tools.vk_wrapper.pfvkCreateImage = (PFN_vkCreateImage)load(device, "vkCreateImage");
  tools.vk_wrapper.pfvkCreateImageView = (PFN_vkCreateImageView)load(device, "vkCreateImageView");
  tools.vk_wrapper.pfvkCreateInstance = (PFN_vkCreateInstance)load(device, "vkCreateInstance");
  tools.vk_wrapper.pfvkCreatePipelineCache = (PFN_vkCreatePipelineCache)load(device, "vkCreatePipelineCache");
  tools.vk_wrapper.pfvkCreatePipelineLayout = (PFN_vkCreatePipelineLayout)load(device, "vkCreatePipelineLayout");
  tools.vk_wrapper.pfvkCreateQueryPool = (PFN_vkCreateQueryPool)load(device, "vkCreateQueryPool");
  tools.vk_wrapper.pfvkCreateRenderPass = (PFN_vkCreateRenderPass)load(device, "vkCreateRenderPass");
  tools.vk_wrapper.pfvkCreateSampler = (PFN_vkCreateSampler)load(device, "vkCreateSampler");
  tools.vk_wrapper.pfvkCreateSemaphore = (PFN_vkCreateSemaphore)load(device, "vkCreateSemaphore");
  tools.vk_wrapper.pfvkCreateShaderModule = (PFN_vkCreateShaderModule)load(device, "vkCreateShaderModule");
  tools.vk_wrapper.vkDestroyBuffer = (PFN_vkDestroyBuffer)load(device, "vkDestroyBuffer");
  tools.vk_wrapper.vkDestroyBufferView = (PFN_vkDestroyBufferView)load(device, "vkDestroyBufferView");
  tools.vk_wrapper.vkDestroyCommandPool = (PFN_vkDestroyCommandPool)load(device, "vkDestroyCommandPool");
  tools.vk_wrapper.vkDestroyDescriptorPool = (PFN_vkDestroyDescriptorPool)load(device, "vkDestroyDescriptorPool");
  tools.vk_wrapper.vkDestroyDescriptorSetLayout = (PFN_vkDestroyDescriptorSetLayout)load(
      device, "vkDestroyDescriptorSetLayout");
  tools.vk_wrapper.vkDestroyDevice = (PFN_vkDestroyDevice)load(device, "vkDestroyDevice");
  tools.vk_wrapper.vkDestroyEvent = (PFN_vkDestroyEvent)load(device, "vkDestroyEvent");
  tools.vk_wrapper.vkDestroyFence = (PFN_vkDestroyFence)load(device, "vkDestroyFence");
  tools.vk_wrapper.vkDestroyFramebuffer = (PFN_vkDestroyFramebuffer)load(device, "vkDestroyFramebuffer");
  tools.vk_wrapper.vkDestroyImage = (PFN_vkDestroyImage)load(device, "vkDestroyImage");
  tools.vk_wrapper.vkDestroyImageView = (PFN_vkDestroyImageView)load(device, "vkDestroyImageView");
  tools.vk_wrapper.vkDestroyInstance = (PFN_vkDestroyInstance)load(device, "vkDestroyInstance");
  tools.vk_wrapper.vkDestroyPipeline = (PFN_vkDestroyPipeline)load(device, "vkDestroyPipeline");
  tools.vk_wrapper.vkDestroyPipelineCache = (PFN_vkDestroyPipelineCache)load(device, "vkDestroyPipelineCache");
  tools.vk_wrapper.vkDestroyPipelineLayout = (PFN_vkDestroyPipelineLayout)load(device, "vkDestroyPipelineLayout");
  tools.vk_wrapper.vkDestroyQueryPool = (PFN_vkDestroyQueryPool)load(device, "vkDestroyQueryPool");
  tools.vk_wrapper.vkDestroyRenderPass = (PFN_vkDestroyRenderPass)load(device, "vkDestroyRenderPass");
  tools.vk_wrapper.vkDestroySampler = (PFN_vkDestroySampler)load(device, "vkDestroySampler");
  tools.vk_wrapper.vkDestroySemaphore = (PFN_vkDestroySemaphore)load(device, "vkDestroySemaphore");
  tools.vk_wrapper.vkDestroyShaderModule = (PFN_vkDestroyShaderModule)load(device, "vkDestroyShaderModule");
  tools.vk_wrapper.pfvkDeviceWaitIdle = (PFN_vkDeviceWaitIdle)load(device, "vkDeviceWaitIdle");
  tools.vk_wrapper.pfvkEndCommandBuffer = (PFN_vkEndCommandBuffer)load(device, "vkEndCommandBuffer");

  tools.vk_wrapper.pfvkFlushMappedMemoryRanges = (PFN_vkFlushMappedMemoryRanges)load(device,
                                                                    "vkFlushMappedMemoryRanges");
  tools.vk_wrapper.vkFreeCommandBuffers  = (PFN_vkFreeCommandBuffers)load(device, "vkFreeCommandBuffers");
  tools.vk_wrapper.pfvkFreeDescriptorSets = (PFN_vkFreeDescriptorSets)load(device, "vkFreeDescriptorSets");
  tools.vk_wrapper.vkFreeMemory = (PFN_vkFreeMemory)load(device, "vkFreeMemory");

  tools.vk_wrapper.vkGetBufferMemoryRequirements = (PFN_vkGetBufferMemoryRequirements)load(
      device, "vkGetBufferMemoryRequirements");
  tools.vk_wrapper.vkGetDeviceMemoryCommitment = (PFN_vkGetDeviceMemoryCommitment)load(
      device, "vkGetDeviceMemoryCommitment");
  tools.vk_wrapper.vkGetDeviceProcAddr = (PFN_vkGetDeviceProcAddr)load(device, "vkGetDeviceProcAddr");
  tools.vk_wrapper.vkGetDeviceQueue = (PFN_vkGetDeviceQueue)load(device, "vkGetDeviceQueue");
  tools.vk_wrapper.pfvkGetEventStatus = (PFN_vkGetEventStatus)load(device, "vkGetEventStatus");
  tools.vk_wrapper.pfvkGetFenceStatus = (PFN_vkGetFenceStatus)load(device, "vkGetFenceStatus");
  tools.vk_wrapper.vkGetImageMemoryRequirements = (PFN_vkGetImageMemoryRequirements)load(
      device, "vkGetImageMemoryRequirements");
  tools.vk_wrapper.vkGetImageSparseMemoryRequirements = (PFN_vkGetImageSparseMemoryRequirements)load(
      device, "vkGetImageSparseMemoryRequirements");
  tools.vk_wrapper.vkGetImageSubresourceLayout = (PFN_vkGetImageSubresourceLayout)load(
      device, "vkGetImageSubresourceLayout");

  tools.vk_wrapper.pfvkGetPipelineCacheData = (PFN_vkGetPipelineCacheData)load(device, "vkGetPipelineCacheData");
  tools.vk_wrapper.pfvkGetQueryPoolResults = (PFN_vkGetQueryPoolResults)load(device, "vkGetQueryPoolResults");
  tools.vk_wrapper.vkGetRenderAreaGranularity = (PFN_vkGetRenderAreaGranularity)load(device,
                                                                    "vkGetRenderAreaGranularity");
  tools.vk_wrapper.pfvkInvalidateMappedMemoryRanges = (PFN_vkInvalidateMappedMemoryRanges)load(
      device, "vkInvalidateMappedMemoryRanges");
  tools.vk_wrapper.pfvkMapMemory = (PFN_vkMapMemory)load(device, "vkMapMemory");
  tools.vk_wrapper.pfvkMergePipelineCaches = (PFN_vkMergePipelineCaches)load(device, "vkMergePipelineCaches");
  tools.vk_wrapper.pfvkQueueBindSparse = (PFN_vkQueueBindSparse)load(device, "vkQueueBindSparse");
  tools.vk_wrapper.pfvkQueueSubmit = (PFN_vkQueueSubmit)load(device, "vkQueueSubmit");
  tools.vk_wrapper.pfvkQueueWaitIdle = (PFN_vkQueueWaitIdle)load(device, "vkQueueWaitIdle");
  tools.vk_wrapper.pfvkResetCommandBuffer = (PFN_vkResetCommandBuffer)load(device, "vkResetCommandBuffer");
  tools.vk_wrapper.pfvkResetCommandPool = (PFN_vkResetCommandPool)load(device, "vkResetCommandPool");
  tools.vk_wrapper.pfvkResetDescriptorPool = (PFN_vkResetDescriptorPool)load(device, "vkResetDescriptorPool");
  tools.vk_wrapper.pfvkResetEvent = (PFN_vkResetEvent)load(device, "vkResetEvent");
  tools.vk_wrapper.pfvkResetFences = (PFN_vkResetFences)load(device, "vkResetFences");
  tools.vk_wrapper.pfvkSetEvent = (PFN_vkSetEvent)load(device, "vkSetEvent");
  tools.vk_wrapper.vkUnmapMemory = (PFN_vkUnmapMemory)load(device, "vkUnmapMemory");
  tools.vk_wrapper.vkUpdateDescriptorSets = (PFN_vkUpdateDescriptorSets)load(device, "vkUpdateDescriptorSets");
  tools.vk_wrapper.pfvkWaitForFences = (PFN_vkWaitForFences)load(device, "vkWaitForFences");
#undef laod
}




#define CONSOLE_COLOR_YELLOW "\x1b[33m"
#define CONSOLE_COLOR_RED "\x1b[31m"
#define CONSOLE_COLOR_RESET "\x1b[0m"
#define CONSOLE_COLOR_FINE "\x1b[2m"

#if defined(VK_EXT_debug_utils)

const char *to_string(VkObjectType type)
{
  switch (type) {

    case VK_OBJECT_TYPE_UNKNOWN:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_UNKNOWN);
    case VK_OBJECT_TYPE_INSTANCE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_INSTANCE);
    case VK_OBJECT_TYPE_PHYSICAL_DEVICE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PHYSICAL_DEVICE);
    case VK_OBJECT_TYPE_DEVICE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEVICE);
    case VK_OBJECT_TYPE_QUEUE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_QUEUE);
    case VK_OBJECT_TYPE_SEMAPHORE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SEMAPHORE);
    case VK_OBJECT_TYPE_COMMAND_BUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_COMMAND_BUFFER);
    case VK_OBJECT_TYPE_FENCE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_FENCE);
    case VK_OBJECT_TYPE_DEVICE_MEMORY:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEVICE_MEMORY);
    case VK_OBJECT_TYPE_BUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER);
    case VK_OBJECT_TYPE_IMAGE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_IMAGE);
    case VK_OBJECT_TYPE_EVENT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_EVENT);
    case VK_OBJECT_TYPE_QUERY_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_QUERY_POOL);
    case VK_OBJECT_TYPE_BUFFER_VIEW:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER_VIEW);
    case VK_OBJECT_TYPE_IMAGE_VIEW:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_IMAGE_VIEW);
    case VK_OBJECT_TYPE_SHADER_MODULE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SHADER_MODULE);
    case VK_OBJECT_TYPE_PIPELINE_CACHE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE_CACHE);
    case VK_OBJECT_TYPE_PIPELINE_LAYOUT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE_LAYOUT);
    case VK_OBJECT_TYPE_RENDER_PASS:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_RENDER_PASS);
    case VK_OBJECT_TYPE_PIPELINE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE);
    case VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT);
    case VK_OBJECT_TYPE_SAMPLER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SAMPLER);
    case VK_OBJECT_TYPE_DESCRIPTOR_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_POOL);
    case VK_OBJECT_TYPE_DESCRIPTOR_SET:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_SET);
    case VK_OBJECT_TYPE_FRAMEBUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_FRAMEBUFFER);
    case VK_OBJECT_TYPE_COMMAND_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_COMMAND_POOL);
    case VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION);
    case VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE);
    case VK_OBJECT_TYPE_SURFACE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SURFACE_KHR);
    case VK_OBJECT_TYPE_SWAPCHAIN_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SWAPCHAIN_KHR);
    case VK_OBJECT_TYPE_DISPLAY_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DISPLAY_KHR);
    case VK_OBJECT_TYPE_DISPLAY_MODE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DISPLAY_MODE_KHR);
    case VK_OBJECT_TYPE_DEBUG_REPORT_CALLBACK_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEBUG_REPORT_CALLBACK_EXT);
#  ifdef VK_ENABLE_BETA_EXTENSIONS
    case VK_OBJECT_TYPE_VIDEO_SESSION_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VIDEO_SESSION_KHR);
#  endif
#  ifdef VK_ENABLE_BETA_EXTENSIONS
    case VK_OBJECT_TYPE_VIDEO_SESSION_PARAMETERS_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VIDEO_SESSION_PARAMETERS_KHR);
#  endif
    case VK_OBJECT_TYPE_CU_MODULE_NVX:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_CU_MODULE_NVX);
    case VK_OBJECT_TYPE_CU_FUNCTION_NVX:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_CU_FUNCTION_NVX);
    case VK_OBJECT_TYPE_DEBUG_UTILS_MESSENGER_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEBUG_UTILS_MESSENGER_EXT);
    case VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR);
    case VK_OBJECT_TYPE_VALIDATION_CACHE_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VALIDATION_CACHE_EXT);
    case VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_NV:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_NV);
    case VK_OBJECT_TYPE_PERFORMANCE_CONFIGURATION_INTEL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PERFORMANCE_CONFIGURATION_INTEL);
    case VK_OBJECT_TYPE_DEFERRED_OPERATION_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEFERRED_OPERATION_KHR);
    case VK_OBJECT_TYPE_INDIRECT_COMMANDS_LAYOUT_NV:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_INDIRECT_COMMANDS_LAYOUT_NV);
    case VK_OBJECT_TYPE_PRIVATE_DATA_SLOT_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PRIVATE_DATA_SLOT_EXT);
    case VK_OBJECT_TYPE_BUFFER_COLLECTION_FUCHSIA:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER_COLLECTION_FUCHSIA);
    default:
      BLI_assert_unreachable();
  }
  return "NotFound";
};


VKAPI_ATTR VkResult VKAPI_CALL
vkCreateDebugUtilsMessengerEXT(VkInstance instance,
                               const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
                               const VkAllocationCallbacks *pAllocator,
                               VkDebugUtilsMessengerEXT *pMessenger)
{
  VkResult r = pfvkCreateDebugUtilsMessengerEXT(instance, pCreateInfo, pAllocator, pMessenger);
  VK_ERROR_CHECK(r, __FUNCTION__);
  return r;
};

/*Supported since Vulkan 1.0.
 * https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_debug_utils.html */
VKAPI_ATTR VkBool32 VKAPI_CALL
debugUtilsCB(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
             VkDebugUtilsMessageTypeFlagsEXT /* messageType*/,
             const VkDebugUtilsMessengerCallbackDataEXT *callbackData,
             void *userData)
{

  VKDebuggingTools *ctx = reinterpret_cast<VKDebuggingTools *>(userData);
  if (ctx->dbgIgnoreMessages.contains(callbackData->messageIdNumber)) {
    return VK_FALSE;
  }

  const bool use_color = CLG_color_support_get(&LOG);
  bool enabled = false;
  if (ELEM(messageSeverity,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)) {
    if ((LOG.type->flag & CLG_FLAG_USE) && (LOG.type->level >= CLG_SEVERITY_INFO)) {
      const char *format = use_color ? CONSOLE_COLOR_FINE "% s\n %s " CONSOLE_COLOR_RESET:" % s\n %s ";
      CLG_logf(LOG.type,
               CLG_SEVERITY_INFO,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
      enabled = true;
    }
  }
  else {

    CLG_Severity clog_severity;
    switch (messageSeverity) {
      case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT:
        clog_severity = CLG_SEVERITY_WARN;
        break;
      case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT:
        clog_severity = CLG_SEVERITY_ERROR;
        break;
      default:
        BLI_assert_unreachable();
    }

    enabled = true;
    if (clog_severity == CLG_SEVERITY_ERROR) {

      const char *format = use_color ? CONSOLE_COLOR_RED "%s\n" CONSOLE_COLOR_RESET " %s \n " : " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);

    }
    else if (LOG.type->level >= CLG_SEVERITY_WARN){
      const char *format = use_color ? CONSOLE_COLOR_YELLOW "%s\n" CONSOLE_COLOR_RESET " %s \n " : " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
    }
  }

  if ((enabled) && ((callbackData->objectCount > 0) || (callbackData->cmdBufLabelCount > 0) ||
                    (callbackData->queueLabelCount > 0))) {
    const size_t str_size = 512;
    char message[str_size];
    memset(message, 0, str_size);

    if (callbackData->objectCount > 0) {
      char tmp_message[500];
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              "\n %d Object[s] related \n" CONSOLE_COLOR_RESET :
                                      "\n %d Object[s] related \n";
      const char *format = use_color ? CONSOLE_COLOR_FINE
                               "ObjectType[%s], Handle[0x%p], Name[%s] \n" CONSOLE_COLOR_RESET :
                                       "ObjectType[%s], Handle[0x%p], Name[%s] \n";

      sprintf(tmp_message, title, callbackData->objectCount);
      strcat(message, tmp_message);
      for (uint32_t object = 0; object < callbackData->objectCount; ++object) {
        sprintf(tmp_message,
                format,
                to_string(callbackData->pObjects[object].objectType),
                (void *)(callbackData->pObjects[object].objectHandle),
                callbackData->pObjects[object].pObjectName);
        strcat(message, tmp_message);
      }
    }
    if (callbackData->cmdBufLabelCount > 0) {
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              " \n %d Command Buffer Label[s] \n " CONSOLE_COLOR_RESET :
                                      " \n %d Command Buffer Label[s] \n ";

      const char *format = use_color ? CONSOLE_COLOR_FINE
                               " CmdLabels[%d]  %s \n" CONSOLE_COLOR_RESET :
                                       " CmdLabels[%d]  %s \n";
      char tmp_message[500];
      sprintf(tmp_message, title, callbackData->cmdBufLabelCount);
      strcat(message, tmp_message);
      for (uint32_t label = 0; label < callbackData->cmdBufLabelCount; ++label) {
        sprintf(tmp_message, format, -(int)label, callbackData->pCmdBufLabels[label].pLabelName);
        strcat(message, tmp_message);
      }
    }
    if (callbackData->queueLabelCount > 0) {
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              "\n % d Queue Label[s] \n " CONSOLE_COLOR_RESET:
                                      "\n % d Queue Label[s] \n ";
      const char *format = use_color ? CONSOLE_COLOR_FINE
                               " QLabels[%d]  %s \n" CONSOLE_COLOR_RESET :
                                       " QLabels[%d]  %s \n";

      char tmp_message[500];
      sprintf(tmp_message, title, callbackData->queueLabelCount);
      strcat(message, tmp_message);
      for (uint32_t label = 0; label < callbackData->queueLabelCount; ++label) {
        sprintf(tmp_message, format, -(int)label, callbackData->pQueueLabels[label].pLabelName);
        strcat(message, tmp_message);
      }
    }
    printf("%s\n", message);
    fflush(stdout);

  }

  return VK_TRUE;
};



static VkResult CreateDebugUtils(VkDebugUtilsMessageSeverityFlagsEXT flag, VKDebuggingTools &deb)
{

  deb.dbgIgnoreMessages.clear();
  BLI_assert(pfvkCreateDebugUtilsMessengerEXT);

  VkDebugUtilsMessengerCreateInfoEXT dbg_messenger_create_info;
  dbg_messenger_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  dbg_messenger_create_info.pNext = nullptr;
  dbg_messenger_create_info.flags = 0;
  dbg_messenger_create_info.messageSeverity = flag;
  dbg_messenger_create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                          VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                          VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  dbg_messenger_create_info.pfnUserCallback = debugUtilsCB;
  dbg_messenger_create_info.pUserData = &deb;
  return pfvkCreateDebugUtilsMessengerEXT(
      deb.instance, &dbg_messenger_create_info, nullptr, &deb.dbgMessenger);
}
static VkResult DestroyDebugUtils(VKDebuggingTools &deb)
{

  BLI_assert(pfvkDestroyDebugUtilsMessengerEXT);
  pfvkDestroyDebugUtilsMessengerEXT(deb.instance, deb.dbgMessenger, nullptr);

  deb.dbgIgnoreMessages.clear();
  deb.dbgMessenger = nullptr;
  deb.instance = VK_NULL_HANDLE;

  return VK_SUCCESS;
}

static bool CreateDebug(VKDebuggingTools &deb)
{
  /*Associate flag settings with interfaces?*/
  VkDebugUtilsMessageSeverityFlagsEXT flag = VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
                                             VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                             VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;

  flag = (VkDebugReportFlagBitsEXT)(flag | VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT);

  CreateDebugUtils(flag, deb);
  return true;
};

#endif

bool init_vk_callbacks(void *instance)
{
  CLOG_ENSURE(&LOG);

  /*
  One-to-one with instances.
  Currently, we do not assume multiple instances.
  */
  BLI_assert((tools.instance == VK_NULL_HANDLE) ||
             ((tools.instance != VK_NULL_HANDLE) && (instance == tools.instance)));
  if (tools.instance != VK_NULL_HANDLE) {
    return true;
  }

  tools.instance = static_cast<VkInstance>(instance);
  DynamicLibInitialize();
  #if defined(VK_EXT_debug_utils)
  BlenderVulkanLoadInstance(tools.instance);
  if (tools.enabled) {
    CreateDebug(tools);
  };
  #endif

  return true;
}

void destroy_vk_callbacks()
{

#if defined(VK_EXT_debug_utils)
  if (tools.enabled) {
    if (tools.dbgMessenger) {
      DestroyDebugUtils(tools);
    }
  }
#endif
  tools.clear();

}

void object_vk_label(VkDevice device, VkObjectType objType, uint64_t obj, const std::string &name)
{
  if ( tools.enabled) {
    VkDebugUtilsObjectNameInfoEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
    info.objectType = objType;
    info.objectHandle = obj;
    info.pObjectName = name.c_str();
    pfvkSetDebugUtilsObjectNameEXT(device, &info);
  }
}

template<> void object_vk_label(VkDevice device, VkInstance obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_INSTANCE, (uint64_t)obj, name_);
}

template<> void object_vk_label(VkDevice device, VkDevice obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DEVICE, (uint64_t)obj, name_);
}

template<> void object_vk_label(VkDevice device, VkPipeline obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_PIPELINE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkFramebuffer obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_FRAMEBUFFER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkImage obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_IMAGE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkSampler obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SAMPLER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkBuffer obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_BUFFER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkSemaphore obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SEMAPHORE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkRenderPass obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_RENDER_PASS, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkFence obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_FENCE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDescriptorSet obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_SET, (uint64_t)obj, name_);
}
template<>
void object_vk_label(VkDevice device, VkDescriptorSetLayout obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkShaderModule obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SHADER_MODULE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkQueue obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_QUEUE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDescriptorPool obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_POOL, (uint64_t)obj, name_);
}
void pushMarker(VkCommandBuffer cmd, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkCmdBeginDebugUtilsLabelEXT(cmd, &info);
  }
}
void setMarker(VkCommandBuffer cmd, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkCmdInsertDebugUtilsLabelEXT(cmd, &info);
  }
}
void popMarker(VkCommandBuffer cmd)
{
  if (tools.enabled) {
    pfvkCmdEndDebugUtilsLabelEXT(cmd);
  }
}
void pushMarker(VkQueue q, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkQueueBeginDebugUtilsLabelEXT(q, &info);
  }
}
void setMarker(VkQueue q, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkQueueInsertDebugUtilsLabelEXT(q, &info);
  }
}
void popMarker(VkQueue q)
{
  if (tools.enabled) {
    pfvkQueueEndDebugUtilsLabelEXT(q);
  }
}
void raise_vk_error(const char *info){

  if (tools.enabled) {

    static VkDebugUtilsMessengerCallbackDataEXT cbdata;
    cbdata.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CALLBACK_DATA_EXT;
    cbdata.pNext = 0;
    cbdata.messageIdNumber = 100;
    cbdata.pMessageIdName = "Raise";
    cbdata.objectCount = 0;
    cbdata.flags = 0;
    cbdata.pObjects = nullptr;
    cbdata.pMessage = info;
    pfvkSubmitDebugUtilsMessageEXT(tools.instance,
                                   VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
                                   VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT,
                                   &cbdata);
  }

}
void check_vk_resources(const char * /* info */){};

}  // namespace debug
}  // namespace gpu
}  // namespace blender
