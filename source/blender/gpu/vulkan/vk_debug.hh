/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */
#pragma once

#include "BKE_global.h"
#include "BLI_set.hh"
#include "BLI_string.h"

#include "vk_common.hh"

#include <mutex>
#include <typeindex>

namespace blender::gpu {
class VKContext;
class VKDevice;

namespace debug {
class VKDebuggingTools {
 public:
  bool enabled = false;
  VkDebugUtilsMessageSeverityFlagsEXT message_severity =
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
  /* Function pointer definitions. */
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
  VKDebuggingTools() = default;
  ~VKDebuggingTools();
  void init(VkInstance vk_instance);
  void deinit();
  bool is_ignore(int32_t id_number);
  VkResult create_messenger(VkInstance vk_instance);
  void destroy_messenger(VkInstance vk_instance);
  void print_labels(const VkDebugUtilsMessengerCallbackDataEXT *callback_data, bool use_color);

 private:
  VkDebugUtilsMessengerEXT vk_debug_utils_messenger = nullptr;
  Set<int32_t> vk_message_id_number_ignored;
  std::mutex ignore_mutex;
  void print_vulkan_version();
  void add_group(int32_t id_number);
  void remove_group(int32_t id_number);
};

void object_label(VkObjectType vk_object_type, uint64_t object_handle, const char *name);
template<typename T> void object_label(T vk_object_type, const char *name)
{
  if (!(G.debug & G_DEBUG_GPU)) {
    return;
  }
  const size_t label_size = 64;
  char label[label_size];
  memset(label, 0, label_size);
  static int stats = 0;
  SNPRINTF(label, "%s_%d", name, stats++);
  object_label(to_vk_object_type(vk_object_type), (uint64_t)vk_object_type, (const char *)label);
};

void push_marker(VkCommandBuffer vk_command_buffer, const char *name);
void set_marker(VkCommandBuffer vk_command_buffer, const char *name);
void pop_marker(VkCommandBuffer vk_command_buffer);
void push_marker(const VKDevice &device, const char *name);
void set_marker(const VKDevice &device, const char *name);
void pop_marker(const VKDevice &device);

/* how to use : debug::raise_message(-12345,VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,"This is
 * a raise message. %llx", (uintptr_t)vk_object); */
template<typename... Args>
void raise_message(int32_t id_number,
                   VkDebugUtilsMessageSeverityFlagBitsEXT vk_severity_flag_bits,
                   const char *fmt,
                   Args... args)
{
  VKContext *context = VKContext::get();
  if (!context) {
    return;
  }
  const VKDevice &device = VKBackend::get().device_get();
  const VKDebuggingTools &debugging_tools = device.debugging_tools_get();
  if (debugging_tools.enabled) {
    char *info = BLI_sprintfN(fmt, args...);
    static VkDebugUtilsMessengerCallbackDataEXT vk_call_back_data;
    vk_call_back_data.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CALLBACK_DATA_EXT;
    vk_call_back_data.pNext = VK_NULL_HANDLE;
    vk_call_back_data.messageIdNumber = id_number;
    vk_call_back_data.pMessageIdName = "VulkanMessenger";
    vk_call_back_data.objectCount = 0;
    vk_call_back_data.flags = 0;
    vk_call_back_data.pObjects = VK_NULL_HANDLE;
    vk_call_back_data.pMessage = info;
    debugging_tools.vkSubmitDebugUtilsMessageEXT_r(device.instance_get(),
                                                   vk_severity_flag_bits,
                                                   VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT,
                                                   &vk_call_back_data);
    MEM_freeN((void *)info);
  }
}
}  // namespace debug
}  // namespace blender::gpu
