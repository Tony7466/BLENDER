/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {

/**
 * `VK_ALLOCATION_CALLBACKS` initializes allocation callbacks for host allocations.
 * The macro creates a local static variable with the name `vk_allocation_callbacks`
 * that can be passed to VULKAN API functions that expect
 * `const VkAllocationCallbacks *pAllocator`.
 *
 * When Blender is compiled with `WITH_VULKAN_GUARDEDALLOC` this will use
 * `MEM_guardedalloc` for host allocations that the driver does on behalf
 * of blender. More internal allocations are still being allocated via the
 * implementation inside the VULKAN device driver.
 *
 * When `WITH_VULKAN_GUARDEDALLOC=Off` the memory allocation implemented
 * in the vulkan device driver is used for both internal and application
 * focused memory operations.
 */

// #define WITH_VULKAN_GUARDEDALLOC_NOTIFICATIONS
// #define WITH_VULKAN_DEVICE_MEMORY_CALLBACKS

#ifdef WITH_VULKAN_GUARDEDALLOC
void *vk_memory_allocation(void *user_data,
                           size_t size,
                           size_t alignment,
                           VkSystemAllocationScope scope);
void *vk_memory_reallocation(
    void *user_data, void *original, size_t size, size_t alignment, VkSystemAllocationScope scope);
void vk_memory_free(void *user_data, void *memory);

#  ifdef WITH_VULKAN_GUARDEDALLOC_NOTIFICATIONS
void vk_memory_allocation_notification(void *pUserData,
                                       size_t size,
                                       VkInternalAllocationType allocationType,
                                       VkSystemAllocationScope allocationScope);

void vk_memory_free_notification(void *pUserData,
                                 size_t size,
                                 VkInternalAllocationType allocationType,
                                 VkSystemAllocationScope allocationScope);
#  endif

constexpr VkAllocationCallbacks vk_allocation_callbacks_init(const char *name)
{
  VkAllocationCallbacks callbacks = {};
  callbacks.pUserData = const_cast<char *>(name);
  callbacks.pfnAllocation = vk_memory_allocation;
  callbacks.pfnReallocation = vk_memory_reallocation;
  callbacks.pfnFree = vk_memory_free;
#  ifdef WITH_VULKAN_GUARDEDALLOC_NOTIFICATIONS
  callbacks.pfnInternalAllocation = vk_memory_allocation_notification;
  callbacks.pfnInternalFree = vk_memory_free_notification;
#  endif
  return callbacks;
}

#  define VK_ALLOCATION_CALLBACKS \
    static constexpr const VkAllocationCallbacks vk_allocation_callbacks_ = \
        vk_allocation_callbacks_init(__FILE__); \
    static constexpr const VkAllocationCallbacks *vk_allocation_callbacks = \
        &vk_allocation_callbacks_;
#else
#  define VK_ALLOCATION_CALLBACKS \
    static constexpr const VkAllocationCallbacks *vk_allocation_callbacks = nullptr;
#endif

#ifdef WITH_VULKAN_DEVICE_MEMORY_CALLBACKS
void vma_device_memory_allocate(VmaAllocator allocator,
                                uint32_t memory_type,
                                VkDeviceMemory memory,
                                VkDeviceSize size,
                                void *user_data);
void vma_device_memory_free(VmaAllocator allocator,
                            uint32_t memory_type,
                            VkDeviceMemory memory,
                            VkDeviceSize size,
                            void *user_data);

constexpr VmaDeviceMemoryCallbacks vma_device_memory_callbacks_init()
{
  VmaDeviceMemoryCallbacks callbacks = {};
  callbacks.pfnAllocate = vma_device_memory_allocate;
  callbacks.pfnFree = vma_device_memory_free;
  return callbacks;
}

#  define VMA_DEVICE_MEMORY_CALLBACKS \
    static constexpr const VmaDeviceMemoryCallbacks vma_device_memory_callbacks_ = \
        vma_device_memory_callbacks_init(); \
    static constexpr const VmaDeviceMemoryCallbacks *vma_device_memory_callbacks = \
        &vma_device_memory_callbacks_;
#else
#  define VMA_DEVICE_MEMORY_CALLBACKS \
    static constexpr const VmaDeviceMemoryCallbacks *vma_device_memory_callbacks = nullptr;
#endif

}  // namespace blender::gpu
