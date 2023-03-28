/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */
#pragma once

#include "BKE_global.h"
#include "BLI_string.h"

#include "vk_common.hh"

#include <typeindex>

namespace blender::gpu {
class VKContext;
namespace debug {
typedef struct VKDebuggingTools {
  bool enabled = false;
  /* Function pointer definitions .*/
  PFN_vkCreateDebugUtilsMessengerEXT vkCreateDebugUtilsMessengerEXT_r = nullptr;
  PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT_r = nullptr;
  PFN_vkSubmitDebugUtilsMessageEXT vkSubmitDebugUtilsMessageEXT_r = nullptr;
  PFN_vkCmdBeginDebugUtilsLabelEXT vkCmdBeginDebugUtilsLabelEXT_r = nullptr;
  PFN_vkCmdEndDebugUtilsLabelEXT vkCmdEndDebugUtilsLabelEXT_r = nullptr;
  PFN_vkCmdInsertDebugUtilsLabelEXT vkCmdInsertDebugUtilsLabelEXT_r = nullptr;
  PFN_vkQueueBeginDebugUtilsLabelEXT vkQueueBeginDebugUtilsLabelEXT_r = nullptr;
  PFN_vkQueueEndDebugUtilsLabelEXT vkQueueEndDebugUtilsLabelEXT_r = nullptr;
  PFN_vkQueueInsertDebugUtilsLabelEXT vkQueueInsertDebugUtilsLabelEXT_r = nullptr;
  PFN_vkSetDebugUtilsObjectNameEXT vkSetDebugUtilsObjectNameEXT_r = nullptr;
  PFN_vkSetDebugUtilsObjectTagEXT vkSetDebugUtilsObjectTagEXT_r = nullptr;

} VKDebuggingTools;
bool init_vk_callbacks(VKContext *ctx, PFN_vkGetInstanceProcAddr instload);
void destroy_vk_callbacks(VKContext *ctx);
void object_vk_label(VKContext *ctx, VkObjectType objType, uint64_t obj, const char *name);

template<typename T> void object_vk_label(VKContext *ctx, T obj, const char *name)
{
  if (!(G.debug & G_DEBUG_GPU)) {
    return;
  }
  const size_t label_size = 64;
  char label[label_size];
  memset(label, 0, label_size);
  static int stats = 0;
  SNPRINTF(label, "%s_%d", name, stats++);
  object_vk_label(ctx, to_vk_object_type(obj), (uint64_t)obj, (const char *)label);
};

void pushMarker(VKContext *ctx, VkCommandBuffer cmd, const char *name);
void setMarker(VKContext *ctx, VkCommandBuffer cmd, const char *name);
void popMarker(VKContext *ctx, VkCommandBuffer cmd);
void pushMarker(VKContext *ctx, VkQueue q, const char *name);
void setMarker(VKContext *ctx, VkQueue q, const char *name);
void popMarker(VKContext *ctx, VkQueue q);

}  // namespace debug
}  // namespace blender::gpu
