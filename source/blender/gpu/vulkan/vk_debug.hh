/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "gl_debug.hh"
#include "GHOST_C-api.h"



namespace blender {
namespace gpu {
namespace debug {

void raise_vk_error(const char *info);
void check_vk_resources(const char *info);

/**
 * This function needs to be called once per context.
 */
bool init_vk_callbacks(void *instance);
void destroy_vk_callbacks();


/* render doc demo => https://
 * github.com/baldurk/renderdoc/blob/52e9404961277d1bb1ed03a376b722c2b91e3762/util/test/demos/vk/vk_test.h#L231
 */
template<typename T> void object_vk_label(VkDevice device, T obj, const std::string &name);
void object_vk_label(VkDevice device, VkObjectType objType, uint64_t obj, const std::string &name);

void pushMarker(VkCommandBuffer cmd, const std::string &name);
void setMarker(VkCommandBuffer cmd, const std::string &name);
void popMarker(VkCommandBuffer cmd);
void pushMarker(VkQueue q, const std::string &name);
void setMarker(VkQueue q, const std::string &name);
void popMarker(VkQueue q);

}
}
}


#define VK_ERROR_CHECK_ENABLE

/* clang-format off */

#ifdef VK_ERROR_CHECK_ENABLE

#define __STR_VK_CHECK(s) "" #s

#define VK_ERROR_CHECK(r,name){\
  if (G.debug & G_DEBUG_GPU) {\
    if (r != VK_SUCCESS) { \
        fprintf(stderr, \
                "Vulkan Error :: %s failled with %s\n", \
                  name,\
                blender::gpu::to_vk_error_string(r) ); \
    }\
}}

#define VK_FUNC_ERROR_CHECK_OVERRIDE(func, ...) \
inline VKAPI_ATTR VkResult VKAPI_CALL func(ARG_LIST(__VA_ARGS__)) \
{ \
  VkResult  r =  ::func(ARG_LIST_CALL(__VA_ARGS__));\
  VK_ERROR_CHECK(r, __STR_VK_CHECK( ::func(__VA_ARGS__)));\
  return r;\
}

VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateInstance,const VkInstanceCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkInstance*, pInstance);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEnumeratePhysicalDevices , VkInstance, instance, uint32_t*, pPhysicalDeviceCount, VkPhysicalDevice* ,pPhysicalDevices);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkGetPhysicalDeviceImageFormatProperties , VkPhysicalDevice ,physicalDevice, VkFormat ,format, VkImageType ,type, VkImageTiling, tiling, VkImageUsageFlags, usage, VkImageCreateFlags ,flags, VkImageFormatProperties* ,pImageFormatProperties);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateDevice , VkPhysicalDevice ,physicalDevice, const VkDeviceCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDevice* ,pDevice);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEnumerateInstanceExtensionProperties , const char* ,pLayerName, uint32_t*, pPropertyCount, VkExtensionProperties*, pProperties);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEnumerateDeviceExtensionProperties , VkPhysicalDevice ,physicalDevice, const char*, pLayerName, uint32_t* ,pPropertyCount, VkExtensionProperties* ,pProperties);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEnumerateInstanceLayerProperties , uint32_t* ,pPropertyCount, VkLayerProperties* ,pProperties);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEnumerateDeviceLayerProperties , VkPhysicalDevice, physicalDevice, uint32_t* ,pPropertyCount, VkLayerProperties*, pProperties);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkQueueSubmit , VkQueue, queue, uint32_t, submitCount, const VkSubmitInfo*, pSubmits, VkFence ,fence);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkQueueWaitIdle , VkQueue, queue);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkDeviceWaitIdle , VkDevice, device);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkAllocateMemory , VkDevice, device, const VkMemoryAllocateInfo*, pAllocateInfo, const VkAllocationCallbacks*, pAllocator, VkDeviceMemory* ,pMemory);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkMapMemory , VkDevice ,device, VkDeviceMemory ,memory, VkDeviceSize ,offset, VkDeviceSize ,size, VkMemoryMapFlags ,flags, void**, ppData);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkFlushMappedMemoryRanges , VkDevice ,device, uint32_t, memoryRangeCount, const VkMappedMemoryRange* ,pMemoryRanges);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkInvalidateMappedMemoryRanges , VkDevice, device, uint32_t ,memoryRangeCount, const VkMappedMemoryRange*, pMemoryRanges);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkBindBufferMemory , VkDevice , device, VkBuffer ,buffer, VkDeviceMemory ,memory, VkDeviceSize, memoryOffset);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkBindImageMemory , VkDevice ,device, VkImage ,image, VkDeviceMemory ,memory, VkDeviceSize ,memoryOffset);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkQueueBindSparse , VkQueue ,queue, uint32_t, bindInfoCount, const VkBindSparseInfo* ,pBindInfo, VkFence, fence);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateFence , VkDevice ,device, const VkFenceCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFence* ,pFence);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkResetFences , VkDevice, device, uint32_t ,fenceCount, const VkFence* ,pFences);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkGetFenceStatus , VkDevice, device, VkFence, fence);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkWaitForFences , VkDevice , device, uint32_t ,fenceCount, const VkFence*, pFences, VkBool32 ,waitAll, uint64_t, timeout);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateSemaphore , VkDevice , device, const VkSemaphoreCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkSemaphore*, pSemaphore);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateEvent , VkDevice , device, const VkEventCreateInfo*, pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkEvent*, pEvent);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkGetEventStatus , VkDevice , device, VkEvent ,event);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkSetEvent , VkDevice , device, VkEvent, event);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkResetEvent , VkDevice , device, VkEvent ,event);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateQueryPool , VkDevice , device, const VkQueryPoolCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkQueryPool* ,pQueryPool);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkGetQueryPoolResults , VkDevice , device, VkQueryPool ,queryPool, uint32_t, firstQuery, uint32_t ,queryCount, size_t, dataSize, void* ,pData, VkDeviceSize, stride, VkQueryResultFlags, flags);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateBuffer , VkDevice , device, const VkBufferCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkBuffer* ,pBuffer);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateBufferView , VkDevice , device, const VkBufferViewCreateInfo*, pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkBufferView*, pView);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateImage , VkDevice , device, const VkImageCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkImage* ,pImage);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateImageView , VkDevice , device, const VkImageViewCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkImageView* ,pView);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateShaderModule , VkDevice , device, const VkShaderModuleCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkShaderModule* ,pShaderModule);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreatePipelineCache , VkDevice , device, const VkPipelineCacheCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkPipelineCache* ,pPipelineCache);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkGetPipelineCacheData , VkDevice , device, VkPipelineCache ,pipelineCache, size_t*, pDataSize, void*, pData);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkMergePipelineCaches , VkDevice , device, VkPipelineCache ,dstCache, uint32_t, srcCacheCount, const VkPipelineCache* ,pSrcCaches);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateGraphicsPipelines , VkDevice , device, VkPipelineCache, pipelineCache, uint32_t ,createInfoCount, const VkGraphicsPipelineCreateInfo*, pCreateInfos, const VkAllocationCallbacks* ,pAllocator, VkPipeline* ,pPipelines);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateComputePipelines , VkDevice , device, VkPipelineCache ,pipelineCache, uint32_t ,createInfoCount, const VkComputePipelineCreateInfo*, pCreateInfos, const VkAllocationCallbacks* ,pAllocator, VkPipeline* ,pPipelines);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreatePipelineLayout , VkDevice , device, const VkPipelineLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkPipelineLayout* ,pPipelineLayout);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateSampler , VkDevice , device, const VkSamplerCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkSampler*, pSampler);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateDescriptorSetLayout , VkDevice , device, const VkDescriptorSetLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorSetLayout* ,pSetLayout);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateDescriptorPool , VkDevice , device, const VkDescriptorPoolCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorPool*, pDescriptorPool);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkResetDescriptorPool , VkDevice , device, VkDescriptorPool, descriptorPool, VkDescriptorPoolResetFlags, flags);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkAllocateDescriptorSets , VkDevice , device, const VkDescriptorSetAllocateInfo* ,pAllocateInfo, VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkFreeDescriptorSets , VkDevice , device, VkDescriptorPool, descriptorPool, uint32_t ,descriptorSetCount, const VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateFramebuffer , VkDevice , device, const VkFramebufferCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFramebuffer* ,pFramebuffer);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateRenderPass , VkDevice , device, const VkRenderPassCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkRenderPass*, pRenderPass);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkCreateCommandPool , VkDevice , device, const VkCommandPoolCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkCommandPool* ,pCommandPool);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkResetCommandPool , VkDevice , device, VkCommandPool, commandPool, VkCommandPoolResetFlags, flags);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkAllocateCommandBuffers , VkDevice , device, const VkCommandBufferAllocateInfo* ,pAllocateInfo, VkCommandBuffer* ,pCommandBuffers);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkBeginCommandBuffer , VkCommandBuffer, commandBuffer, const VkCommandBufferBeginInfo*, pBeginInfo);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkEndCommandBuffer , VkCommandBuffer ,commandBuffer);
VK_FUNC_ERROR_CHECK_OVERRIDE(vkResetCommandBuffer , VkCommandBuffer ,commandBuffer, VkCommandBufferResetFlags, flags);




#endif

/* clang-format on */


