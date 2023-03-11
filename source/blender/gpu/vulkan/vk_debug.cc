/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 *
 * Debug features of Vulkan.
 */

#include "BLI_set.hh"
#include "BLI_system.h"
#include  "BKE_global.h"
#include  "CLG_log.h"
#include  "GPU_debug.h"
#include  "GPU_platform.h"

#include  "vk_common.hh"
#include  "vk_debug.hh"
#include  "vk_context.hh"

#ifdef VK_WRAPPER_IMPL
#  error "should not be defined VK_WRAPPER_IMPL"
#endif

static CLG_LogRef LOG = {"gpu.debug.vulkan"};

/*@link  https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_debug_utils.html  */
#ifdef VK_EXT_debug_utils
#define VK_DEBUG_ENABLED 1
#endif

/*for creating breakpoints.*/
extern inline void VK_ERROR_CHECK(VkResult r, const char *name)
{
  if (r != VK_SUCCESS) {
    fprintf(
        stderr, "Vulkan Error :: %s failled with %s\n", name, blender::gpu::to_vk_error_string(r));
  }
};




namespace blender::gpu::debug {

struct VKDebuggingTools {
/* Function pointer definitions .*/
#if defined(VK_DEBUG_ENABLED)
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
#endif /* defined(VK_DEBUG_ENABLED) */

  bool enabled = false;
  /*One -on -one for instance.*/
  VkInstance instance = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkDebugUtilsMessengerEXT dbgMessenger = nullptr;
  Set<int32_t> dbgIgnoreMessages;
  std::mutex lists_mutex_;

  VKDebuggingTools()
  {
    clear();
  }
  void clear()
  {
    instance = VK_NULL_HANDLE;
    device = VK_NULL_HANDLE;
    dbgIgnoreMessages.clear();
    dbgMessenger = nullptr;
    enabled = false;
  }
  void add_ignore(int32_t id)
  {
    lists_mutex_.lock();
    dbgIgnoreMessages.add(id);
    lists_mutex_.unlock();
  }
  void remove_ignore(int32_t id)
  {
    lists_mutex_.lock();
    dbgIgnoreMessages.remove(id);
    lists_mutex_.unlock();
  }
};
/*If we don't have multiple instances, VKDebuggingTools is Singleton.*/
static VKDebuggingTools tools;
}  // namespace blender::gpu::debug

#define CONSOLE_COLOR_YELLOW "\x1b[33m"
#define CONSOLE_COLOR_RED "\x1b[31m"
#define CONSOLE_COLOR_RESET "\x1b[0m"
#define CONSOLE_COLOR_FINE "\x1b[2m"


namespace blender {
namespace gpu {

const char *to_string(VkObjectType type)
{
  switch (type) {

    case VK_OBJECT_TYPE_UNKNOWN:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_UNKNOWN);
    case VK_OBJECT_TYPE_INSTANCE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_INSTANCE);
    case VK_OBJECT_TYPE_PHYSICAL_DEVICE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PHYSICAL_DEVICE);
    case VK_OBJECT_TYPE_DEVICE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEVICE);
    case VK_OBJECT_TYPE_QUEUE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_QUEUE);
    case VK_OBJECT_TYPE_SEMAPHORE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SEMAPHORE);
    case VK_OBJECT_TYPE_COMMAND_BUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_COMMAND_BUFFER);
    case VK_OBJECT_TYPE_FENCE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_FENCE);
    case VK_OBJECT_TYPE_DEVICE_MEMORY:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEVICE_MEMORY);
    case VK_OBJECT_TYPE_BUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER);
    case VK_OBJECT_TYPE_IMAGE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_IMAGE);
    case VK_OBJECT_TYPE_EVENT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_EVENT);
    case VK_OBJECT_TYPE_QUERY_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_QUERY_POOL);
    case VK_OBJECT_TYPE_BUFFER_VIEW:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER_VIEW);
    case VK_OBJECT_TYPE_IMAGE_VIEW:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_IMAGE_VIEW);
    case VK_OBJECT_TYPE_SHADER_MODULE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SHADER_MODULE);
    case VK_OBJECT_TYPE_PIPELINE_CACHE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE_CACHE);
    case VK_OBJECT_TYPE_PIPELINE_LAYOUT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE_LAYOUT);
    case VK_OBJECT_TYPE_RENDER_PASS:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_RENDER_PASS);
    case VK_OBJECT_TYPE_PIPELINE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PIPELINE);
    case VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT);
    case VK_OBJECT_TYPE_SAMPLER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SAMPLER);
    case VK_OBJECT_TYPE_DESCRIPTOR_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_POOL);
    case VK_OBJECT_TYPE_DESCRIPTOR_SET:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_SET);
    case VK_OBJECT_TYPE_FRAMEBUFFER:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_FRAMEBUFFER);
    case VK_OBJECT_TYPE_COMMAND_POOL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_COMMAND_POOL);
    case VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION);
    case VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE);
    case VK_OBJECT_TYPE_SURFACE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SURFACE_KHR);
    case VK_OBJECT_TYPE_SWAPCHAIN_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_SWAPCHAIN_KHR);
    case VK_OBJECT_TYPE_DISPLAY_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DISPLAY_KHR);
    case VK_OBJECT_TYPE_DISPLAY_MODE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DISPLAY_MODE_KHR);
    case VK_OBJECT_TYPE_DEBUG_REPORT_CALLBACK_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEBUG_REPORT_CALLBACK_EXT);
#ifdef VK_ENABLE_BETA_EXTENSIONS
    case VK_OBJECT_TYPE_VIDEO_SESSION_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VIDEO_SESSION_KHR);
#endif
#ifdef VK_ENABLE_BETA_EXTENSIONS
    case VK_OBJECT_TYPE_VIDEO_SESSION_PARAMETERS_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VIDEO_SESSION_PARAMETERS_KHR);
#endif
    case VK_OBJECT_TYPE_CU_MODULE_NVX:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_CU_MODULE_NVX);
    case VK_OBJECT_TYPE_CU_FUNCTION_NVX:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_CU_FUNCTION_NVX);
    case VK_OBJECT_TYPE_DEBUG_UTILS_MESSENGER_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEBUG_UTILS_MESSENGER_EXT);
    case VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR);
    case VK_OBJECT_TYPE_VALIDATION_CACHE_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_VALIDATION_CACHE_EXT);
    case VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_NV:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_NV);
    case VK_OBJECT_TYPE_PERFORMANCE_CONFIGURATION_INTEL:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PERFORMANCE_CONFIGURATION_INTEL);
    case VK_OBJECT_TYPE_DEFERRED_OPERATION_KHR:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_DEFERRED_OPERATION_KHR);
    case VK_OBJECT_TYPE_INDIRECT_COMMANDS_LAYOUT_NV:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_INDIRECT_COMMANDS_LAYOUT_NV);
    case VK_OBJECT_TYPE_PRIVATE_DATA_SLOT_EXT:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_PRIVATE_DATA_SLOT_EXT);
    case VK_OBJECT_TYPE_BUFFER_COLLECTION_FUCHSIA:
      return __STR_VK_CHECK(VK_OBJECT_TYPE_BUFFER_COLLECTION_FUCHSIA);
    default:
      BLI_assert_unreachable();
  }
  return "NotFound";
};

namespace debug {

void object_vk_label(VkDevice device, VkObjectType objType, uint64_t obj, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsObjectNameInfoEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
    info.objectType = objType;
    info.objectHandle = obj;
    info.pObjectName = name.c_str();
    tools.vkSetDebugUtilsObjectNameEXT_r(device, &info);
  }
}

template<> void object_vk_label(VkDevice device, VkInstance obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_INSTANCE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDevice obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DEVICE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkPipeline obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_PIPELINE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkFramebuffer obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_FRAMEBUFFER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkImage obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_IMAGE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkSampler obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SAMPLER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkBuffer obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_BUFFER, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkSemaphore obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SEMAPHORE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkRenderPass obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_RENDER_PASS, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkFence obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_FENCE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDescriptorSet obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_SET, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDescriptorSetLayout obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkShaderModule obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_SHADER_MODULE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkQueue obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_QUEUE, (uint64_t)obj, name_);
}
template<> void object_vk_label(VkDevice device, VkDescriptorPool obj, const std::string &name)
{
  static int stat = 0;
  auto name_ = name + "_" + std::to_string(stat++);
  object_vk_label(device, VK_OBJECT_TYPE_DESCRIPTOR_POOL, (uint64_t)obj, name_);
}
void pushMarker(VkCommandBuffer cmd, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    tools.vkCmdBeginDebugUtilsLabelEXT_r(cmd, &info);
  }
}
void setMarker(VkCommandBuffer cmd, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    tools.vkCmdInsertDebugUtilsLabelEXT_r(cmd, &info);
  }
}
void popMarker(VkCommandBuffer cmd)
{
  if (tools.enabled) {
    tools.vkCmdEndDebugUtilsLabelEXT_r(cmd);
  }
}
void pushMarker(VkQueue q, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    tools.vkQueueBeginDebugUtilsLabelEXT_r(q, &info);
  }
}
void setMarker(VkQueue q, const std::string &name)
{
  if (tools.enabled) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    tools.vkQueueInsertDebugUtilsLabelEXT_r(q, &info);
  }
}
void popMarker(VkQueue q)
{
  if (tools.enabled) {
    tools.vkQueueEndDebugUtilsLabelEXT_r(q);
  }
}
void raise_vk_error(const char *info)
{

  if (tools.enabled) {

    static VkDebugUtilsMessengerCallbackDataEXT cbdata;
    cbdata.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CALLBACK_DATA_EXT;
    cbdata.pNext = 0;
    cbdata.messageIdNumber = 100;
    cbdata.pMessageIdName = "Raise";
    cbdata.objectCount = 0;
    cbdata.flags = 0;
    cbdata.pObjects = nullptr;
    cbdata.pMessage = info;
    tools.vkSubmitDebugUtilsMessageEXT_r(tools.instance,
                                   VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
                                   VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT,
                                   &cbdata);
  }
}
void check_vk_resources(const char * /* info */){};

#if defined(VK_DEBUG_ENABLED)

static void vulkan_dynamic_debug_functions(VkInstance &instance,
                                           PFN_vkGetInstanceProcAddr instload)
{

#  if defined(VK_DEBUG_ENABLED)
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

  if (tools.vkCmdBeginDebugUtilsLabelEXT_r != nullptr) {
    tools.enabled = true;
  }

#  endif /* defined(VK_DEBUG_ENABLED) */
}

VKAPI_ATTR VkResult VKAPI_CALL
vkCreateDebugUtilsMessengerEXT(VkInstance instance,
                               const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
                               const VkAllocationCallbacks *pAllocator,
                               VkDebugUtilsMessengerEXT *pMessenger)
{
  VkResult r = tools.vkCreateDebugUtilsMessengerEXT_r(
      instance, pCreateInfo, pAllocator, pMessenger);
  VK_ERROR_CHECK(r, __FUNCTION__);
  return r;
};


VKAPI_ATTR VkBool32 VKAPI_CALL
debugUtilsCB(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
             VkDebugUtilsMessageTypeFlagsEXT /* messageType*/,
             const VkDebugUtilsMessengerCallbackDataEXT *callbackData,
             void *userData)
{

  VKDebuggingTools *ctx = reinterpret_cast<VKDebuggingTools *>(userData);
  if (ctx->dbgIgnoreMessages.contains(callbackData->messageIdNumber)) {
    return VK_FALSE;
  }

  const bool use_color = CLG_color_support_get(&LOG);
  bool enabled = false;
  if (ELEM(messageSeverity,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)) {
    if ((LOG.type->flag & CLG_FLAG_USE) && (LOG.type->level >= CLG_SEVERITY_INFO)) {
      const char *format = use_color ? CONSOLE_COLOR_FINE "% s\n %s " CONSOLE_COLOR_RESET :
                                       " % s\n %s ";
      CLG_logf(LOG.type,
               CLG_SEVERITY_INFO,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
      enabled = true;
    }
  }
  else {

    CLG_Severity clog_severity;
    switch (messageSeverity) {
      case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT:
        clog_severity = CLG_SEVERITY_WARN;
        break;
      case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT:
        clog_severity = CLG_SEVERITY_ERROR;
        break;
      default:
        BLI_assert_unreachable();
    }

    enabled = true;
    if (clog_severity == CLG_SEVERITY_ERROR) {

      const char *format = use_color ? CONSOLE_COLOR_RED "%s\n" CONSOLE_COLOR_RESET " %s \n " :
                                       " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
    }
    else if (LOG.type->level >= CLG_SEVERITY_WARN) {
      const char *format = use_color ? CONSOLE_COLOR_YELLOW "%s\n" CONSOLE_COLOR_RESET " %s \n " :
                                       " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
    }
  }

  if ((enabled) && ((callbackData->objectCount > 0) || (callbackData->cmdBufLabelCount > 0) ||
                    (callbackData->queueLabelCount > 0))) {
    const size_t str_size = 512;
    char message[str_size];
    memset(message, 0, str_size);

    if (callbackData->objectCount > 0) {
      char tmp_message[500];
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              "\n %d Object[s] related \n" CONSOLE_COLOR_RESET :
                                      "\n %d Object[s] related \n";
      const char *format = use_color ? CONSOLE_COLOR_FINE
                               "ObjectType[%s], Handle[0x%p], Name[%s] \n" CONSOLE_COLOR_RESET :
                                       "ObjectType[%s], Handle[0x%p], Name[%s] \n";

      sprintf(tmp_message, title, callbackData->objectCount);
      strcat(message, tmp_message);
      for (uint32_t object = 0; object < callbackData->objectCount; ++object) {
        sprintf(tmp_message,
                format,
                to_string(callbackData->pObjects[object].objectType),
                (void *)(callbackData->pObjects[object].objectHandle),
                callbackData->pObjects[object].pObjectName);
        strcat(message, tmp_message);
      }
    }
    if (callbackData->cmdBufLabelCount > 0) {
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              " \n %d Command Buffer Label[s] \n " CONSOLE_COLOR_RESET :
                                      " \n %d Command Buffer Label[s] \n ";

      const char *format = use_color ? CONSOLE_COLOR_FINE
                               " CmdLabels[%d]  %s \n" CONSOLE_COLOR_RESET :
                                       " CmdLabels[%d]  %s \n";
      char tmp_message[500];
      sprintf(tmp_message, title, callbackData->cmdBufLabelCount);
      strcat(message, tmp_message);
      for (uint32_t label = 0; label < callbackData->cmdBufLabelCount; ++label) {
        sprintf(tmp_message, format, -(int)label, callbackData->pCmdBufLabels[label].pLabelName);
        strcat(message, tmp_message);
      }
    }
    if (callbackData->queueLabelCount > 0) {
      const char *title = use_color ? CONSOLE_COLOR_FINE
                              "\n % d Queue Label[s] \n " CONSOLE_COLOR_RESET :
                                      "\n % d Queue Label[s] \n ";
      const char *format = use_color ? CONSOLE_COLOR_FINE
                               " QLabels[%d]  %s \n" CONSOLE_COLOR_RESET :
                                       " QLabels[%d]  %s \n";

      char tmp_message[500];
      sprintf(tmp_message, title, callbackData->queueLabelCount);
      strcat(message, tmp_message);
      for (uint32_t label = 0; label < callbackData->queueLabelCount; ++label) {
        sprintf(tmp_message, format, -(int)label, callbackData->pQueueLabels[label].pLabelName);
        strcat(message, tmp_message);
      }
    }
    printf("%s\n", message);
    fflush(stdout);
  }

  return VK_TRUE;
};

static VkResult CreateDebugUtils(VkDebugUtilsMessageSeverityFlagsEXT flag, VKDebuggingTools &deb)
{

  deb.dbgIgnoreMessages.clear();
  BLI_assert(tools.vkCreateDebugUtilsMessengerEXT_r);

  VkDebugUtilsMessengerCreateInfoEXT dbg_messenger_create_info;
  dbg_messenger_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  dbg_messenger_create_info.pNext = nullptr;
  dbg_messenger_create_info.flags = 0;
  dbg_messenger_create_info.messageSeverity = flag;
  dbg_messenger_create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                          VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                          VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  dbg_messenger_create_info.pfnUserCallback = debugUtilsCB;
  dbg_messenger_create_info.pUserData = &deb;
  return tools.vkCreateDebugUtilsMessengerEXT_r(
      deb.instance, &dbg_messenger_create_info, nullptr, &deb.dbgMessenger);
}
static VkResult DestroyDebugUtils(VKDebuggingTools &deb)
{

  BLI_assert(tools.vkDestroyDebugUtilsMessengerEXT_r);
  tools.vkDestroyDebugUtilsMessengerEXT_r(deb.instance, deb.dbgMessenger, nullptr);

  deb.dbgIgnoreMessages.clear();
  deb.dbgMessenger = nullptr;
  deb.instance = VK_NULL_HANDLE;

  return VK_SUCCESS;
}

static bool CreateDebug(VKDebuggingTools &deb)
{
  /*Associate flag settings with interfaces?*/
  VkDebugUtilsMessageSeverityFlagsEXT flag = VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
                                             VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                             VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;

  flag = (VkDebugReportFlagBitsEXT)(flag | VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT);

  CreateDebugUtils(flag, deb);
  return true;
};

#endif

bool init_vk_callbacks(void *instance,PFN_vkGetInstanceProcAddr instload)
{
  CLOG_ENSURE(&LOG);

  /*
  One-to-one with instances.
  Currently, we do not assume multiple instances.
  */
  BLI_assert((tools.instance == VK_NULL_HANDLE) ||
             ((tools.instance != VK_NULL_HANDLE) && (instance == tools.instance)));
  if (tools.instance != VK_NULL_HANDLE) {
    return true;
  }

  tools.instance = static_cast<VkInstance>(instance);

#if defined(VK_DEBUG_ENABLED)
  if (instload) {
    VkInstance vk_instance = static_cast<VkInstance>(instance);
    vulkan_dynamic_debug_functions(vk_instance, instload);
    if (tools.enabled) {
      CreateDebug(tools);
      return true;
    };
  }
#endif
  return false;
}

void destroy_vk_callbacks()
{

#if defined(VK_DEBUG_ENABLED)
  if (tools.enabled) {
    if (tools.dbgMessenger) {
      DestroyDebugUtils(tools);
    }
  }
#endif
  tools.clear();
}

}  // namespace debug
}  // namespace gpu
}  // namespace blender

namespace blender {
namespace tests {
void test_create()
{
  VkApplicationInfo app_info = {};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "Blender_test";
  app_info.applicationVersion = VK_MAKE_VERSION(3, 5, 0);
  app_info.pEngineName = "Blender_test";
  app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  app_info.apiVersion = VK_MAKE_VERSION(1, 2, 0);

  VkInstanceCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  create_info.pApplicationInfo = &app_info;
  create_info.enabledLayerCount = 0;
  create_info.ppEnabledLayerNames = nullptr;
  create_info.enabledExtensionCount = 0;
  create_info.ppEnabledExtensionNames = nullptr;
  VkInstance instance = VK_NULL_HANDLE;
  vkCreateInstance(&create_info, NULL, &instance);
  vkDestroyInstance(instance, nullptr);
}
}  // namespace test
}  // namespace blender
