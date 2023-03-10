/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once
#ifndef VK_DEBUG_H
#  define VK_DEBUG_H

#include "BKE_global.h"
#include "vk_common.hh"
#include "gl_debug.hh"



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

void vulkanLoadDevice(VkDevice& device);
}
}
}


#define VK_ERROR_CHECK_ENABLE

/* clang-format off */

#ifdef VK_ERROR_CHECK_ENABLE



#define VK_FUNC_ERROR_CHECK_DEFINE(func, ...) \
VKAPI_ATTR VkResult VKAPI_CALL func(ARG_LIST(__VA_ARGS__)); 


VK_FUNC_ERROR_CHECK_DEFINE(vkCreateInstance,const VkInstanceCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkInstance*, pInstance);
VK_FUNC_ERROR_CHECK_DEFINE(vkEnumeratePhysicalDevices , VkInstance, instance, uint32_t*, pPhysicalDeviceCount, VkPhysicalDevice* ,pPhysicalDevices);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetPhysicalDeviceImageFormatProperties , VkPhysicalDevice ,physicalDevice, VkFormat ,format, VkImageType ,type, VkImageTiling, tiling, VkImageUsageFlags, usage, VkImageCreateFlags ,flags, VkImageFormatProperties* ,pImageFormatProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateDevice , VkPhysicalDevice ,physicalDevice, const VkDeviceCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDevice* ,pDevice);
VK_FUNC_ERROR_CHECK_DEFINE(vkEnumerateInstanceExtensionProperties , const char* ,pLayerName, uint32_t*, pPropertyCount, VkExtensionProperties*, pProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkEnumerateDeviceExtensionProperties , VkPhysicalDevice ,physicalDevice, const char*, pLayerName, uint32_t* ,pPropertyCount, VkExtensionProperties* ,pProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkEnumerateInstanceLayerProperties , uint32_t* ,pPropertyCount, VkLayerProperties* ,pProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkEnumerateDeviceLayerProperties , VkPhysicalDevice, physicalDevice, uint32_t* ,pPropertyCount, VkLayerProperties*, pProperties);
VK_FUNC_ERROR_CHECK_DEFINE(vkQueueSubmit , VkQueue, queue, uint32_t, submitCount, const VkSubmitInfo*, pSubmits, VkFence ,fence);
VK_FUNC_ERROR_CHECK_DEFINE(vkQueueWaitIdle , VkQueue, queue);
VK_FUNC_ERROR_CHECK_DEFINE(vkDeviceWaitIdle , VkDevice, device);
VK_FUNC_ERROR_CHECK_DEFINE(vkAllocateMemory , VkDevice, device, const VkMemoryAllocateInfo*, pAllocateInfo, const VkAllocationCallbacks*, pAllocator, VkDeviceMemory* ,pMemory);
VK_FUNC_ERROR_CHECK_DEFINE(vkMapMemory , VkDevice ,device, VkDeviceMemory ,memory, VkDeviceSize ,offset, VkDeviceSize ,size, VkMemoryMapFlags ,flags, void**, ppData);
VK_FUNC_ERROR_CHECK_DEFINE(vkFlushMappedMemoryRanges , VkDevice ,device, uint32_t, memoryRangeCount, const VkMappedMemoryRange* ,pMemoryRanges);
VK_FUNC_ERROR_CHECK_DEFINE(vkInvalidateMappedMemoryRanges , VkDevice, device, uint32_t ,memoryRangeCount, const VkMappedMemoryRange*, pMemoryRanges);
VK_FUNC_ERROR_CHECK_DEFINE(vkBindBufferMemory , VkDevice , device, VkBuffer ,buffer, VkDeviceMemory ,memory, VkDeviceSize, memoryOffset);
VK_FUNC_ERROR_CHECK_DEFINE(vkBindImageMemory , VkDevice ,device, VkImage ,image, VkDeviceMemory ,memory, VkDeviceSize ,memoryOffset);
VK_FUNC_ERROR_CHECK_DEFINE(vkQueueBindSparse , VkQueue ,queue, uint32_t, bindInfoCount, const VkBindSparseInfo* ,pBindInfo, VkFence, fence);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateFence , VkDevice ,device, const VkFenceCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFence* ,pFence);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetFences , VkDevice, device, uint32_t ,fenceCount, const VkFence* ,pFences);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetFenceStatus , VkDevice, device, VkFence, fence);
VK_FUNC_ERROR_CHECK_DEFINE(vkWaitForFences , VkDevice , device, uint32_t ,fenceCount, const VkFence*, pFences, VkBool32 ,waitAll, uint64_t, timeout);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateSemaphore , VkDevice , device, const VkSemaphoreCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkSemaphore*, pSemaphore);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateEvent , VkDevice , device, const VkEventCreateInfo*, pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkEvent*, pEvent);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetEventStatus , VkDevice , device, VkEvent ,event);
VK_FUNC_ERROR_CHECK_DEFINE(vkSetEvent , VkDevice , device, VkEvent, event);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetEvent , VkDevice , device, VkEvent ,event);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateQueryPool , VkDevice , device, const VkQueryPoolCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkQueryPool* ,pQueryPool);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetQueryPoolResults , VkDevice , device, VkQueryPool ,queryPool, uint32_t, firstQuery, uint32_t ,queryCount, size_t, dataSize, void* ,pData, VkDeviceSize, stride, VkQueryResultFlags, flags);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateBuffer , VkDevice , device, const VkBufferCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkBuffer* ,pBuffer);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateBufferView , VkDevice , device, const VkBufferViewCreateInfo*, pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkBufferView*, pView);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateImage , VkDevice , device, const VkImageCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkImage* ,pImage);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateImageView , VkDevice , device, const VkImageViewCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkImageView* ,pView);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateShaderModule , VkDevice , device, const VkShaderModuleCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkShaderModule* ,pShaderModule);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreatePipelineCache , VkDevice , device, const VkPipelineCacheCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkPipelineCache* ,pPipelineCache);
VK_FUNC_ERROR_CHECK_DEFINE(vkGetPipelineCacheData , VkDevice , device, VkPipelineCache ,pipelineCache, size_t*, pDataSize, void*, pData);
VK_FUNC_ERROR_CHECK_DEFINE(vkMergePipelineCaches , VkDevice , device, VkPipelineCache ,dstCache, uint32_t, srcCacheCount, const VkPipelineCache* ,pSrcCaches);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateGraphicsPipelines , VkDevice , device, VkPipelineCache, pipelineCache, uint32_t ,createInfoCount, const VkGraphicsPipelineCreateInfo*, pCreateInfos, const VkAllocationCallbacks* ,pAllocator, VkPipeline* ,pPipelines);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateComputePipelines , VkDevice , device, VkPipelineCache ,pipelineCache, uint32_t ,createInfoCount, const VkComputePipelineCreateInfo*, pCreateInfos, const VkAllocationCallbacks* ,pAllocator, VkPipeline* ,pPipelines);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreatePipelineLayout , VkDevice , device, const VkPipelineLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkPipelineLayout* ,pPipelineLayout);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateSampler , VkDevice , device, const VkSamplerCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkSampler*, pSampler);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateDescriptorSetLayout , VkDevice , device, const VkDescriptorSetLayoutCreateInfo* ,pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorSetLayout* ,pSetLayout);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateDescriptorPool , VkDevice , device, const VkDescriptorPoolCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkDescriptorPool*, pDescriptorPool);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetDescriptorPool , VkDevice , device, VkDescriptorPool, descriptorPool, VkDescriptorPoolResetFlags, flags);
VK_FUNC_ERROR_CHECK_DEFINE(vkAllocateDescriptorSets , VkDevice , device, const VkDescriptorSetAllocateInfo* ,pAllocateInfo, VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_ERROR_CHECK_DEFINE(vkFreeDescriptorSets , VkDevice , device, VkDescriptorPool, descriptorPool, uint32_t ,descriptorSetCount, const VkDescriptorSet* ,pDescriptorSets);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateFramebuffer , VkDevice , device, const VkFramebufferCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkFramebuffer* ,pFramebuffer);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateRenderPass , VkDevice , device, const VkRenderPassCreateInfo* ,pCreateInfo, const VkAllocationCallbacks*, pAllocator, VkRenderPass*, pRenderPass);
VK_FUNC_ERROR_CHECK_DEFINE(vkCreateCommandPool , VkDevice , device, const VkCommandPoolCreateInfo*, pCreateInfo, const VkAllocationCallbacks* ,pAllocator, VkCommandPool* ,pCommandPool);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetCommandPool , VkDevice , device, VkCommandPool, commandPool, VkCommandPoolResetFlags, flags);
VK_FUNC_ERROR_CHECK_DEFINE(vkAllocateCommandBuffers , VkDevice , device, const VkCommandBufferAllocateInfo* ,pAllocateInfo, VkCommandBuffer* ,pCommandBuffers);
VK_FUNC_ERROR_CHECK_DEFINE(vkBeginCommandBuffer , VkCommandBuffer, commandBuffer, const VkCommandBufferBeginInfo*, pBeginInfo);
VK_FUNC_ERROR_CHECK_DEFINE(vkEndCommandBuffer , VkCommandBuffer ,commandBuffer);
VK_FUNC_ERROR_CHECK_DEFINE(vkResetCommandBuffer , VkCommandBuffer ,commandBuffer, VkCommandBufferResetFlags, flags);


#define VK_FUNC_VOID_DEFINE(func, ...) \
VKAPI_ATTR void VKAPI_CALL func(ARG_LIST(__VA_ARGS__)); 


VK_FUNC_VOID_DEFINE(vkDestroyInstance,VkInstance, instance,const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyInstance,VkInstance, instance,
                                               const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceFeatures,VkPhysicalDevice, physicalDevice,
                                                         VkPhysicalDeviceFeatures *,pFeatures);
VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceFormatProperties,
    VkPhysicalDevice ,physicalDevice, VkFormat ,format, VkFormatProperties *,pFormatProperties);

VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceProperties,
    VkPhysicalDevice ,physicalDevice, VkPhysicalDeviceProperties *,pProperties);
VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceQueueFamilyProperties,
    VkPhysicalDevice ,physicalDevice,
    uint32_t *,pQueueFamilyPropertyCount,
    VkQueueFamilyProperties *,pQueueFamilyProperties);
VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceMemoryProperties,
    VkPhysicalDevice ,physicalDevice, VkPhysicalDeviceMemoryProperties *,pMemoryProperties);

typedef PFN_vkVoidFunction(VKAPI_PTR *PFN_vkGetDeviceProcAddr)(VkDevice device, const char *pName);

VK_FUNC_VOID_DEFINE(vkDestroyDevice,VkDevice, device,
                                             const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_DEFINE(vkGetDeviceQueue,VkDevice ,device,
                                              uint32_t ,queueFamilyIndex,
                                              uint32_t ,queueIndex,
                                              VkQueue *,pQueue);

VK_FUNC_VOID_DEFINE(vkFreeMemory,VkDevice, device,
                                          VkDeviceMemory, memory,
                                          const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkUnmapMemory,VkDevice ,device, VkDeviceMemory ,memory);
VK_FUNC_VOID_DEFINE(vkGetDeviceMemoryCommitment,VkDevice, device,
                                                         VkDeviceMemory ,memory,
                                                         VkDeviceSize *,pCommittedMemoryInBytes);
VK_FUNC_VOID_DEFINE(vkGetBufferMemoryRequirements,
    VkDevice ,device, VkBuffer ,buffer, VkMemoryRequirements *,pMemoryRequirements);
VK_FUNC_VOID_DEFINE(vkGetImageMemoryRequirements,
    VkDevice ,device, VkImage, image, VkMemoryRequirements *,pMemoryRequirements);
VK_FUNC_VOID_DEFINE(vkGetImageSparseMemoryRequirements,
    VkDevice ,device,
    VkImage ,image,
    uint32_t *,pSparseMemoryRequirementCount,
    VkSparseImageMemoryRequirements *,pSparseMemoryRequirements);
VK_FUNC_VOID_DEFINE(vkGetPhysicalDeviceSparseImageFormatProperties,
    VkPhysicalDevice ,physicalDevice,
    VkFormat, format,
    VkImageType ,type,
    VkSampleCountFlagBits, samples,
    VkImageUsageFlags, usage,
    VkImageTiling ,tiling,
    uint32_t *,pPropertyCount,
    VkSparseImageFormatProperties *,pProperties);

VK_FUNC_VOID_DEFINE(vkDestroyFence,VkDevice, device,
                                            VkFence, fence,
                                            const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroySemaphore,VkDevice, device,
                                                VkSemaphore, semaphore,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyEvent,VkDevice ,device,
                                            VkEvent ,event,
                                            const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyQueryPool,VkDevice ,device,
                                                VkQueryPool ,queryPool,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyBuffer,VkDevice ,device,
                                             VkBuffer ,buffer,
                                             const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyBufferView,VkDevice, device,
                                                 VkBufferView ,bufferView,
                                                 const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyImage,VkDevice ,device,
                                            VkImage, image,
                                            const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_DEFINE(vkGetImageSubresourceLayout,VkDevice ,device,
                                                         VkImage ,image,
                                                         const VkImageSubresource *,pSubresource,
                                                         VkSubresourceLayout *,pLayout);

VK_FUNC_VOID_DEFINE(vkDestroyImageView,VkDevice ,device,
                                                VkImageView, imageView,
                                                const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyShaderModule,VkDevice ,device,
                                                   VkShaderModule ,shaderModule,
                                                   const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyPipelineCache,VkDevice, device,
                                                    VkPipelineCache ,pipelineCache,
                                                    const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyPipeline,VkDevice ,device,
                                               VkPipeline ,pipeline,
                                               const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyPipelineLayout,VkDevice ,device,
                                                     VkPipelineLayout ,pipelineLayout,
                                                     const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroySampler,VkDevice ,device,
                                              VkSampler ,sampler,
                                              const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyDescriptorSetLayout,
    VkDevice ,device,
    VkDescriptorSetLayout, descriptorSetLayout,
    const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyDescriptorPool,VkDevice ,device,
                                                     VkDescriptorPool ,descriptorPool,
                                                     const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_DEFINE(vkUpdateDescriptorSets,VkDevice ,device,
                                                    uint32_t ,descriptorWriteCount,
                                                    const VkWriteDescriptorSet *,pDescriptorWrites,
                                                    uint32_t ,descriptorCopyCount,
                                                    const VkCopyDescriptorSet *,pDescriptorCopies);

VK_FUNC_VOID_DEFINE(vkDestroyFramebuffer,VkDevice ,device,
                                                  VkFramebuffer ,framebuffer,
                                                  const VkAllocationCallbacks *,pAllocator);

VK_FUNC_VOID_DEFINE(vkDestroyRenderPass,VkDevice ,device,
                                                 VkRenderPass ,renderPass,
                                                 const VkAllocationCallbacks *,pAllocator);
VK_FUNC_VOID_DEFINE(vkGetRenderAreaGranularity,VkDevice, device,
                                                        VkRenderPass, renderPass,
                                                        VkExtent2D *,pGranularity);

VK_FUNC_VOID_DEFINE(vkDestroyCommandPool,VkDevice ,device,
                                                  VkCommandPool ,commandPool,
                                                  const VkAllocationCallbacks *,pAllocator);


VK_FUNC_VOID_DEFINE(vkFreeCommandBuffers,VkDevice, device,
                                                  VkCommandPool, commandPool,
                                                  uint32_t ,commandBufferCount,
                                                  const VkCommandBuffer *,pCommandBuffers);



VK_FUNC_VOID_DEFINE(vkCmdBindPipeline,VkCommandBuffer, commandBuffer,
                                               VkPipelineBindPoint ,pipelineBindPoint,
                                               VkPipeline ,pipeline);
VK_FUNC_VOID_DEFINE(vkCmdSetViewport,VkCommandBuffer, commandBuffer,
                                              uint32_t ,firstViewport,
                                              uint32_t ,viewportCount,
                                              const VkViewport *,pViewports);
VK_FUNC_VOID_DEFINE(vkCmdSetScissor,VkCommandBuffer ,commandBuffer,
                                             uint32_t ,firstScissor,
                                             uint32_t ,scissorCount,
                                             const VkRect2D *,pScissors);
VK_FUNC_VOID_DEFINE(vkCmdSetLineWidth,VkCommandBuffer ,commandBuffer, float ,lineWidth);
VK_FUNC_VOID_DEFINE(vkCmdSetDepthBias,VkCommandBuffer, commandBuffer,
                                               float ,depthBiasConstantFactor,
                                               float ,depthBiasClamp,
                                               float ,depthBiasSlopeFactor);
VK_FUNC_VOID_DEFINE(vkCmdSetBlendConstants,VkCommandBuffer, commandBuffer, const float*, blendConstants);

VK_FUNC_VOID_DEFINE(vkCmdSetDepthBounds,VkCommandBuffer ,commandBuffer,
                                                 float ,minDepthBounds,
                                                 float, maxDepthBounds);
VK_FUNC_VOID_DEFINE(vkCmdSetStencilCompareMask,VkCommandBuffer ,commandBuffer,
                                                        VkStencilFaceFlags ,faceMask,
                                                        uint32_t ,compareMask);
VK_FUNC_VOID_DEFINE(vkCmdSetStencilWriteMask,VkCommandBuffer, commandBuffer,
                                                      VkStencilFaceFlags ,faceMask,
                                                      uint32_t, writeMask);
VK_FUNC_VOID_DEFINE(vkCmdSetStencilReference,VkCommandBuffer ,commandBuffer,
                                                      VkStencilFaceFlags ,faceMask,
                                                      uint32_t, reference);
VK_FUNC_VOID_DEFINE(vkCmdBindDescriptorSets,VkCommandBuffer ,commandBuffer,
                                                     VkPipelineBindPoint ,pipelineBindPoint,
                                                     VkPipelineLayout, layout,
                                                     uint32_t ,firstSet,
                                                     uint32_t ,descriptorSetCount,
                                                     const VkDescriptorSet *,pDescriptorSets,
                                                     uint32_t ,dynamicOffsetCount,
                                                     const uint32_t *,pDynamicOffsets);
VK_FUNC_VOID_DEFINE(vkCmdBindIndexBuffer,VkCommandBuffer ,commandBuffer,
                                                  VkBuffer ,buffer,
                                                  VkDeviceSize ,offset,
                                                  VkIndexType ,indexType);
VK_FUNC_VOID_DEFINE(vkCmdBindVertexBuffers,VkCommandBuffer, commandBuffer,
                                                    uint32_t ,firstBinding,
                                                    uint32_t ,bindingCount,
                                                    const VkBuffer *,pBuffers,
                                                    const VkDeviceSize *,pOffsets);
VK_FUNC_VOID_DEFINE(vkCmdDraw,VkCommandBuffer, commandBuffer,
                                       uint32_t ,vertexCount,
                                       uint32_t ,instanceCount,
                                       uint32_t ,firstVertex,
                                       uint32_t ,firstInstance);
VK_FUNC_VOID_DEFINE(vkCmdDrawIndexed,VkCommandBuffer ,commandBuffer,
                                              uint32_t, indexCount,
                                              uint32_t ,instanceCount,
                                              uint32_t ,firstIndex,
                                              int32_t ,vertexOffset,
                                              uint32_t, firstInstance);
VK_FUNC_VOID_DEFINE(vkCmdDrawIndirect,VkCommandBuffer ,commandBuffer,
                                               VkBuffer ,buffer,
                                               VkDeviceSize ,offset,
                                               uint32_t ,drawCount,
                                               uint32_t ,stride);
VK_FUNC_VOID_DEFINE(vkCmdDrawIndexedIndirect,VkCommandBuffer ,commandBuffer,
                                                      VkBuffer ,buffer,
                                                      VkDeviceSize, offset,
                                                      uint32_t ,drawCount,
                                                      uint32_t ,stride);
VK_FUNC_VOID_DEFINE(vkCmdDispatch,VkCommandBuffer, commandBuffer,
                                           uint32_t ,groupCountX,
                                           uint32_t ,groupCountY,
                                           uint32_t ,groupCountZ);
VK_FUNC_VOID_DEFINE(vkCmdDispatchIndirect,VkCommandBuffer ,commandBuffer,
                                                   VkBuffer ,buffer,
                                                   VkDeviceSize ,offset);
VK_FUNC_VOID_DEFINE(vkCmdCopyBuffer,VkCommandBuffer, commandBuffer,
                                             VkBuffer ,srcBuffer,
                                             VkBuffer ,dstBuffer,
                                             uint32_t ,regionCount,
                                             const VkBufferCopy *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdCopyImage,VkCommandBuffer ,commandBuffer,
                                            VkImage ,srcImage,
                                            VkImageLayout ,srcImageLayout,
                                            VkImage ,dstImage,
                                            VkImageLayout, dstImageLayout,
                                            uint32_t ,regionCount,
                                            const VkImageCopy *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdBlitImage,VkCommandBuffer ,commandBuffer,
                                            VkImage ,srcImage,
                                            VkImageLayout ,srcImageLayout,
                                            VkImage ,dstImage,
                                            VkImageLayout ,dstImageLayout,
                                            uint32_t ,regionCount,
                                            const VkImageBlit *,pRegions,
                                            VkFilter ,filter);
VK_FUNC_VOID_DEFINE(vkCmdCopyBufferToImage,VkCommandBuffer, commandBuffer,
                                                    VkBuffer ,srcBuffer,
                                                    VkImage ,dstImage,
                                                    VkImageLayout ,dstImageLayout,
                                                    uint32_t ,regionCount,
                                                    const VkBufferImageCopy *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdCopyImageToBuffer,VkCommandBuffer, commandBuffer,
                                                    VkImage ,srcImage,
                                                    VkImageLayout ,srcImageLayout,
                                                    VkBuffer ,dstBuffer,
                                                    uint32_t ,regionCount,
                                                    const VkBufferImageCopy *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdUpdateBuffer,VkCommandBuffer, commandBuffer,
                                               VkBuffer, dstBuffer,
                                               VkDeviceSize ,dstOffset,
                                               VkDeviceSize ,dataSize,
                                               const void *,pData);
VK_FUNC_VOID_DEFINE(vkCmdFillBuffer,VkCommandBuffer ,commandBuffer,
                                             VkBuffer ,dstBuffer,
                                             VkDeviceSize ,dstOffset,
                                             VkDeviceSize ,size,
                                             uint32_t ,data);

VK_FUNC_VOID_DEFINE(vkCmdClearColorImage,VkCommandBuffer, commandBuffer,
                                                  VkImage ,image,
                                                  VkImageLayout ,imageLayout,
                                                  const VkClearColorValue *,pColor,
                                                  uint32_t ,rangeCount,
                                                  const VkImageSubresourceRange *,pRanges);

VK_FUNC_VOID_DEFINE(vkCmdClearDepthStencilImage,
    VkCommandBuffer ,commandBuffer,
    VkImage ,image,
    VkImageLayout, imageLayout,
    const VkClearDepthStencilValue *,pDepthStencil,
    uint32_t ,rangeCount,
    const VkImageSubresourceRange *,pRanges);
VK_FUNC_VOID_DEFINE(vkCmdClearAttachments,VkCommandBuffer ,commandBuffer,
                                                   uint32_t, attachmentCount,
                                                   const VkClearAttachment *,pAttachments,
                                                   uint32_t, rectCount,
                                                   const VkClearRect *,pRects);
VK_FUNC_VOID_DEFINE(vkCmdResolveImage,VkCommandBuffer, commandBuffer,
                                               VkImage ,srcImage,
                                               VkImageLayout ,srcImageLayout,
                                               VkImage ,dstImage,
                                               VkImageLayout ,dstImageLayout,
                                               uint32_t ,regionCount,
                                               const VkImageResolve *,pRegions);
VK_FUNC_VOID_DEFINE(vkCmdSetEvent,VkCommandBuffer ,commandBuffer,
                                           VkEvent ,event,
                                           VkPipelineStageFlags ,stageMask);
VK_FUNC_VOID_DEFINE(vkCmdResetEvent,VkCommandBuffer ,commandBuffer,
                                             VkEvent ,event,
                                             VkPipelineStageFlags ,stageMask);
VK_FUNC_VOID_DEFINE(vkCmdWaitEvents,VkCommandBuffer ,commandBuffer,
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
VK_FUNC_VOID_DEFINE(vkCmdPipelineBarrier,
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
VK_FUNC_VOID_DEFINE(vkCmdBeginQuery,VkCommandBuffer, commandBuffer,
                                             VkQueryPool ,queryPool,
                                             uint32_t ,query,
                                             VkQueryControlFlags ,flags);
VK_FUNC_VOID_DEFINE(vkCmdEndQuery,VkCommandBuffer, commandBuffer,
                                           VkQueryPool ,queryPool,
                                           uint32_t ,query);
VK_FUNC_VOID_DEFINE(vkCmdResetQueryPool,VkCommandBuffer, commandBuffer,
                                                 VkQueryPool ,queryPool,
                                                 uint32_t ,firstQuery,
                                                 uint32_t ,queryCount);
VK_FUNC_VOID_DEFINE(vkCmdWriteTimestamp,VkCommandBuffer ,commandBuffer,
                                                 VkPipelineStageFlagBits ,pipelineStage,
                                                 VkQueryPool ,queryPool,
                                                 uint32_t ,query);
VK_FUNC_VOID_DEFINE(vkCmdCopyQueryPoolResults,VkCommandBuffer, commandBuffer,
                                                       VkQueryPool, queryPool,
                                                       uint32_t ,firstQuery,
                                                       uint32_t ,queryCount,
                                                       VkBuffer ,dstBuffer,
                                                       VkDeviceSize ,dstOffset,
                                                       VkDeviceSize ,stride,
                                                       VkQueryResultFlags ,flags);
VK_FUNC_VOID_DEFINE(vkCmdPushConstants,VkCommandBuffer ,commandBuffer,
                                                VkPipelineLayout ,layout,
                                                VkShaderStageFlags, stageFlags,
                                                uint32_t ,offset,
                                                uint32_t ,size,
                                                const void *,pValues);
VK_FUNC_VOID_DEFINE(vkCmdBeginRenderPass,VkCommandBuffer ,commandBuffer,
                                                  const VkRenderPassBeginInfo *,pRenderPassBegin,
                                                  VkSubpassContents ,contents);
VK_FUNC_VOID_DEFINE(vkCmdNextSubpass,VkCommandBuffer, commandBuffer,
                                              VkSubpassContents ,contents);
VK_FUNC_VOID_DEFINE(vkCmdEndRenderPass,VkCommandBuffer, commandBuffer);
VK_FUNC_VOID_DEFINE(vkCmdExecuteCommands,VkCommandBuffer, commandBuffer,
                                                  uint32_t ,commandBufferCount,
                                                  const VkCommandBuffer *,pCommandBuffers);

#endif

/* clang-format on */


#endif
