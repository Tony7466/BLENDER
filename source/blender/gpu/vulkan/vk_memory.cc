/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_memory.hh"

#include "MEM_guardedalloc.h"

namespace blender::gpu {

#ifdef WITH_VULKAN_GUARDEDALLOC

void *vk_memory_allocation(void *user_data,
                           size_t size,
                           size_t alignment,
                           VkSystemAllocationScope /*scope*/)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  if (alignment) {
    return MEM_mallocN_aligned(size, alignment, name);
  }
  return MEM_mallocN(size, name);
}

void *vk_memory_reallocation(void *user_data,
                             void *original,
                             size_t size,
                             size_t /*alignment*/,
                             VkSystemAllocationScope /*scope*/)
{
  const char *name = static_cast<const char *>(const_cast<const void *>(user_data));
  return MEM_reallocN_id(original, size, name);
}

void vk_memory_free(void * /*user_data*/, void *memory)
{
  if (memory != nullptr) {
    MEM_freeN(memory);
  }
}

#endif

uint32_t Std430::component_mem_size(const shader::Type /*type*/)
{
  return 4;
}

uint32_t Std430::element_alignment(const shader::Type type)
{
  switch (type) {
    case shader::Type::FLOAT:
    case shader::Type::UINT:
    case shader::Type::INT:
    case shader::Type::BOOL:
      return 4;
    case shader::Type::VEC2:
    case shader::Type::UVEC2:
    case shader::Type::IVEC2:
      return 8;
    case shader::Type::VEC3:
    case shader::Type::UVEC3:
    case shader::Type::IVEC3:
    case shader::Type::VEC4:
    case shader::Type::UVEC4:
    case shader::Type::IVEC4:
    case shader::Type::MAT3:
    case shader::Type::MAT4:
      return 16;
    default:
      BLI_assert_msg(false, "Type not supported as push constant");
  }
  return 0;
}

uint32_t Std430::element_components_len(const shader::Type type)
{
  switch (type) {
    case shader::Type::FLOAT:
    case shader::Type::UINT:
    case shader::Type::INT:
    case shader::Type::BOOL:
      return 1;
    case shader::Type::VEC2:
    case shader::Type::UVEC2:
    case shader::Type::IVEC2:
      return 2;
    case shader::Type::VEC3:
    case shader::Type::UVEC3:
    case shader::Type::IVEC3:
    case shader::Type::VEC4:
    case shader::Type::UVEC4:
    case shader::Type::IVEC4:
      return 4;
    case shader::Type::MAT3:
      return 12;
    case shader::Type::MAT4:
      return 16;
    default:
      BLI_assert_msg(false, "Type not supported as push constant");
  }
  return 0;
}

}  // namespace blender::gpu
