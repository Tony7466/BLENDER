/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_memory.hh"

#include "MEM_guardedalloc.h"

namespace blender::gpu {

#ifdef WITH_VULKAN_GUARDEDALLOC

static const char *to_string(const VkSystemAllocationScope vk_system_allocation_scope)
{
  switch (vk_system_allocation_scope) {
    case VK_SYSTEM_ALLOCATION_SCOPE_CACHE:
      return "cache";
    case VK_SYSTEM_ALLOCATION_SCOPE_COMMAND:
      return "command";
    case VK_SYSTEM_ALLOCATION_SCOPE_DEVICE:
      return "device";
    case VK_SYSTEM_ALLOCATION_SCOPE_INSTANCE:
      return "instance";
    case VK_SYSTEM_ALLOCATION_SCOPE_OBJECT:
      return "object";
    default:
      break;
  }
  return "";
}

static void vk_memory_trace(const char *func,
                            const char *name,
                            VkSystemAllocationScope vk_system_allocation_scope,
                            size_t size)
{
  std::cout << func << "(";
  std::cout << "name=" << name;
  std::cout << ",scope=" << to_string(vk_system_allocation_scope);
  std::cout << ",size=" << size;
  std::cout << ")\n";
}

static void vk_memory_trace(const char *func, const char *name)
{
  std::cout << func << "(";
  std::cout << "name=" << name;
  std::cout << ")\n";
}

void *vk_memory_allocation(void *user_data,
                           size_t size,
                           size_t alignment,
                           VkSystemAllocationScope scope)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  vk_memory_trace(__func__, name, scope, size);

  if (alignment) {
    return MEM_mallocN_aligned(size, alignment, name);
  }
  return MEM_mallocN(size, name);
}

void *vk_memory_reallocation(void *user_data,
                             void *original,
                             size_t size,
                             size_t /*alignment*/,
                             VkSystemAllocationScope scope)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  vk_memory_trace(__func__, name, scope, size);
  return MEM_reallocN_id(original, size, name);
}

void vk_memory_free(void * /*user_data*/, void *memory)
{
  if (memory != nullptr) {
    const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
    vk_memory_trace(__func__, name);
    MEM_freeN(memory);
  }
}

#  ifdef WITH_VULKAN_GUARDEDALLOC_NOTIFICATIONS
void vk_memory_allocation_notification(void *user_data,
                                       size_t size,
                                       VkInternalAllocationType /*allocation_type*/,
                                       VkSystemAllocationScope allocation_scope)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  vk_memory_trace(__func__, name, allocation_scope, size);
}

void vk_memory_free_notification(void *user_data,
                                 size_t size,
                                 VkInternalAllocationType /*allocation_type*/,
                                 VkSystemAllocationScope allocation_scope)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  vk_memory_trace(__func__, name, allocation_scope, size);
}
#  endif

#endif

#ifdef WITH_VULKAN_DEVICE_MEMORY_CALLBACKS
void vma_device_memory_allocate(VmaAllocator /*allocator*/,
                                uint32_t /*memory_type*/,
                                VkDeviceMemory /* memory*/,
                                VkDeviceSize size,
                                void *VMA_NULLABLE /* user_data*/)
{
  std::cout << __func__ << "(";
  std::cout << "size=" << size;
  std::cout << ")\n";
}

void vma_device_memory_free(VmaAllocator /*allocator*/,
                            uint32_t /*memory_type*/,
                            VkDeviceMemory /* memory*/,
                            VkDeviceSize size,
                            void *VMA_NULLABLE /* user_data*/)
{
  std::cout << __func__ << "(";
  std::cout << "size=" << size;
  std::cout << ")\n";
}

#endif

}  // namespace blender::gpu
