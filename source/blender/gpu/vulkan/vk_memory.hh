/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

#include "gpu_shader_create_info.hh"

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

#ifdef WITH_VULKAN_GUARDEDALLOC
void *vk_memory_allocation(void *user_data,
                           size_t size,
                           size_t alignment,
                           VkSystemAllocationScope scope);
void *vk_memory_reallocation(
    void *user_data, void *original, size_t size, size_t alignment, VkSystemAllocationScope scope);
void vk_memory_free(void *user_data, void *memory);

constexpr VkAllocationCallbacks vk_allocation_callbacks_init(const char *name)
{
  VkAllocationCallbacks callbacks = {};
  callbacks.pUserData = const_cast<char *>(name);
  callbacks.pfnAllocation = vk_memory_allocation;
  callbacks.pfnReallocation = vk_memory_reallocation;
  callbacks.pfnFree = vk_memory_free;
  callbacks.pfnInternalAllocation = nullptr;
  callbacks.pfnInternalFree = nullptr;
  return callbacks;
}

#  define VK_ALLOCATION_CALLBACKS \
    static constexpr const VkAllocationCallbacks vk_allocation_callbacks_ = \
        vk_allocation_callbacks_init(__func__); \
    static constexpr const VkAllocationCallbacks *vk_allocation_callbacks = \
        &vk_allocation_callbacks_;
#else
#  define VK_ALLOCATION_CALLBACKS \
    static constexpr const VkAllocationCallbacks *vk_allocation_callbacks = nullptr;
#endif

/**
 * Information about alignment/components and memory size for types when using std430 layout.
 */
struct Std430 {
  /** Get the memory size in bytes of a single component using by the given type.*/
  static uint32_t component_mem_size(const shader::Type type);
  /** Get to alignment of the given type in bytes.*/
  static uint32_t element_alignment(const shader::Type type);
  /** Get the number of components that should be allocated for the given type.*/
  static uint32_t element_components_len(const shader::Type type);
};

/**
 * Information about alignment/components and memory size for types when using std140 layout.
 */
struct Std140 {
  /** Get the memory size in bytes of a single component using by the given type.*/
  static uint32_t component_mem_size(const shader::Type type);
  /** Get to alignment of the given type in bytes.*/
  static uint32_t element_alignment(const shader::Type type);
  /** Get the number of components that should be allocated for the given type.*/
  static uint32_t element_components_len(const shader::Type type);
};

}  // namespace blender::gpu
