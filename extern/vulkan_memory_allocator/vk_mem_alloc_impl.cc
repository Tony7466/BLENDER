/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

#ifdef __APPLE__
#  include <MoltenVK/vk_mvk_moltenvk.h>
#else
#  include <vulkan/vulkan.h>
#endif

#define VMA_IMPLEMENTATION
#ifdef DEBUG
#define VMA_ASSERT(test) 
#endif

#include "vk_mem_alloc.h"
