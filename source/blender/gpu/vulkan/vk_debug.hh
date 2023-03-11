/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */
#pragma once

#include "BKE_global.h"
#include "gl_debug.hh"
#include "vk_common.hh"


namespace blender {
namespace gpu {

const char *to_string(VkObjectType type);

namespace debug {

void raise_vk_error(const char *info);
void check_vk_resources(const char *info);

/**
 * This function needs to be called once per context.
 */
bool init_vk_callbacks(void *instance, PFN_vkGetInstanceProcAddr instload);
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

/* clang-format off */
#  define __STR_VK_CHECK(s) "" #s
/* clang-format on */
namespace blender {
namespace tests {
void test_create();
}
}  // namespace  blender
