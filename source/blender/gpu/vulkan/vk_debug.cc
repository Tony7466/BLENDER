/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "BKE_global.h"
#include "CLG_log.h"

#include "vk_debug.hh"
#include "vk_backend.hh"
#include "vk_common.hh"
#include "vk_context.hh"

static CLG_LogRef LOG = {"gpu.debug.vulkan"};

namespace blender::gpu {
void VKContext::debug_group_begin(const char *, int)
{
}

void VKContext::debug_group_end()
{
}

bool VKContext::debug_capture_begin()
{
  return VKBackend::get().debug_capture_begin(vk_instance_);
}

bool VKBackend::debug_capture_begin(VkInstance vk_instance)
{
#ifdef WITH_RENDERDOC
  return renderdoc_api_.start_frame_capture(vk_instance, nullptr);
#else
  UNUSED_VARS(vk_instance);
  return false;
#endif
}

void VKContext::debug_capture_end()
{
  VKBackend::get().debug_capture_end(vk_instance_);
}

void VKBackend::debug_capture_end(VkInstance vk_instance)
{
#ifdef WITH_RENDERDOC
  renderdoc_api_.end_frame_capture(vk_instance, nullptr);
#else
  UNUSED_VARS(vk_instance);
#endif
}

void *VKContext::debug_capture_scope_create(const char * /*name*/)
{
  return nullptr;
}

bool VKContext::debug_capture_scope_begin(void * /*scope*/)
{
  return false;
}

void VKContext::debug_capture_scope_end(void * /*scope*/)
{
}

}  // namespace blender::gpu

namespace blender::gpu::debug {

static void vulkan_dynamic_debug_functions(VKContext *ctx, PFN_vkGetInstanceProcAddr instload)
{
  VKDebuggingTools &tools = ctx->debuggingtools_get();
  VkInstance instance = ctx->instance_get();

  if (instload) {

    tools.enabled = false;
    tools.vkCmdBeginDebugUtilsLabelEXT_r = (PFN_vkCmdBeginDebugUtilsLabelEXT)instload(
        instance, "vkCmdBeginDebugUtilsLabelEXT");
    tools.vkCmdEndDebugUtilsLabelEXT_r = (PFN_vkCmdEndDebugUtilsLabelEXT)instload(
        instance, "vkCmdEndDebugUtilsLabelEXT");
    tools.vkCmdInsertDebugUtilsLabelEXT_r = (PFN_vkCmdInsertDebugUtilsLabelEXT)instload(
        instance, "vkCmdInsertDebugUtilsLabelEXT");
    tools.vkCreateDebugUtilsMessengerEXT_r = (PFN_vkCreateDebugUtilsMessengerEXT)instload(
        instance, "vkCreateDebugUtilsMessengerEXT");
    tools.vkDestroyDebugUtilsMessengerEXT_r = (PFN_vkDestroyDebugUtilsMessengerEXT)instload(
        instance, "vkDestroyDebugUtilsMessengerEXT");
    tools.vkQueueBeginDebugUtilsLabelEXT_r = (PFN_vkQueueBeginDebugUtilsLabelEXT)instload(
        instance, "vkQueueBeginDebugUtilsLabelEXT");
    tools.vkQueueEndDebugUtilsLabelEXT_r = (PFN_vkQueueEndDebugUtilsLabelEXT)instload(
        instance, "vkQueueEndDebugUtilsLabelEXT");
    tools.vkQueueInsertDebugUtilsLabelEXT_r = (PFN_vkQueueInsertDebugUtilsLabelEXT)instload(
        instance, "vkQueueInsertDebugUtilsLabelEXT");
    tools.vkSetDebugUtilsObjectNameEXT_r = (PFN_vkSetDebugUtilsObjectNameEXT)instload(
        instance, "vkSetDebugUtilsObjectNameEXT");
    tools.vkSetDebugUtilsObjectTagEXT_r = (PFN_vkSetDebugUtilsObjectTagEXT)instload(
        instance, "vkSetDebugUtilsObjectTagEXT");
    tools.vkSubmitDebugUtilsMessageEXT_r = (PFN_vkSubmitDebugUtilsMessageEXT)instload(
        instance, "vkSubmitDebugUtilsMessageEXT");
    if (tools.vkCmdBeginDebugUtilsLabelEXT_r) {
      tools.enabled = true;
    }
  }
  else {
    tools.vkCmdBeginDebugUtilsLabelEXT_r = nullptr;
    tools.vkCmdEndDebugUtilsLabelEXT_r = nullptr;
    tools.vkCmdInsertDebugUtilsLabelEXT_r = nullptr;
    tools.vkCreateDebugUtilsMessengerEXT_r = nullptr;
    tools.vkDestroyDebugUtilsMessengerEXT_r = nullptr;
    tools.vkQueueBeginDebugUtilsLabelEXT_r = nullptr;
    tools.vkQueueEndDebugUtilsLabelEXT_r = nullptr;
    tools.vkQueueInsertDebugUtilsLabelEXT_r = nullptr;
    tools.vkSetDebugUtilsObjectNameEXT_r = nullptr;
    tools.vkSetDebugUtilsObjectTagEXT_r = nullptr;
    tools.vkSubmitDebugUtilsMessageEXT_r = nullptr;
    tools.enabled = false;
  }
}

bool init_vk_callbacks(VKContext *ctx, PFN_vkGetInstanceProcAddr instload)
{
  CLOG_ENSURE(&LOG);
  if (instload) {
    vulkan_dynamic_debug_functions(ctx, instload);
    return true;
  };
  return false;
}

void destroy_vk_callbacks(VKContext *ctx)
{
  VKDebuggingTools tools = ctx->debuggingtools_get();
  if (tools.enabled) {
    vulkan_dynamic_debug_functions(ctx, nullptr);
  }
}

void object_vk_label(blender::gpu::VKContext *ctx,
                     VkObjectType objType,
                     uint64_t obj,
                     const char *name)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      VkDebugUtilsObjectNameInfoEXT info = {};
      info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
      info.objectType = objType;
      info.objectHandle = obj;
      info.pObjectName = name;
      tools.vkSetDebugUtilsObjectNameEXT_r(ctx->device_get(), &info);
    }
  }
}

void pushMarker(blender::gpu::VKContext *ctx, VkCommandBuffer cmd, const char *name)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      VkDebugUtilsLabelEXT info = {};
      info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
      info.pLabelName = name;
      tools.vkCmdBeginDebugUtilsLabelEXT_r(cmd, &info);
    }
  }
}

void setMarker(blender::gpu::VKContext *ctx, VkCommandBuffer cmd, const char *name)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      VkDebugUtilsLabelEXT info = {};
      info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
      info.pLabelName = name;
      tools.vkCmdInsertDebugUtilsLabelEXT_r(cmd, &info);
    }
  }
}

void popMarker(blender::gpu::VKContext *ctx, VkCommandBuffer cmd)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      tools.vkCmdEndDebugUtilsLabelEXT_r(cmd);
    }
  }
}

void pushMarker(blender::gpu::VKContext *ctx, VkQueue q, const char *name)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      VkDebugUtilsLabelEXT info = {};
      info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
      info.pLabelName = name;
      tools.vkQueueBeginDebugUtilsLabelEXT_r(q, &info);
    }
  }
}

void setMarker(blender::gpu::VKContext *ctx, VkQueue q, const char *name)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      VkDebugUtilsLabelEXT info = {};
      info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
      info.pLabelName = name;
      tools.vkQueueInsertDebugUtilsLabelEXT_r(q, &info);
    }
  }
}

void popMarker(blender::gpu::VKContext *ctx, VkQueue q)
{
  if (G.debug & G_DEBUG_GPU) {
    VKDebuggingTools tools = ctx->debuggingtools_get();
    if (tools.enabled) {
      tools.vkQueueEndDebugUtilsLabelEXT_r(q);
    }
  }
}

}  // namespace blender::gpu::debug
