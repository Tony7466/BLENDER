/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 *
 * Debug features of Vulkan.
 */


#include  "BLI_set.hh"
#include "BLI_system.h"

#include  "BKE_global.h"

#include  "GPU_debug.h"
#include  "GPU_platform.h"

#include  "CLG_log.h"

#include  "vk_common.hh"
#include  "vk_debug.hh"
#include  "vk_context.hh"

#include  "GHOST_C-api.h"


static CLG_LogRef LOG = {"gpu.debug.vulkan"};

/* Function pointer definitions for use only in this file.*/
#if defined(VK_EXT_debug_utils)
PFN_vkCreateDebugUtilsMessengerEXT  pfvkCreateDebugUtilsMessengerEXT  = nullptr;
PFN_vkDestroyDebugUtilsMessengerEXT pfvkDestroyDebugUtilsMessengerEXT = nullptr;
PFN_vkSubmitDebugUtilsMessageEXT pfvkSubmitDebugUtilsMessageEXT = nullptr;
PFN_vkCmdBeginDebugUtilsLabelEXT pfvkCmdBeginDebugUtilsLabelEXT =nullptr;
PFN_vkCmdEndDebugUtilsLabelEXT pfvkCmdEndDebugUtilsLabelEXT = nullptr;
PFN_vkCmdInsertDebugUtilsLabelEXT pfvkCmdInsertDebugUtilsLabelEXT = nullptr;
PFN_vkQueueBeginDebugUtilsLabelEXT pfvkQueueBeginDebugUtilsLabelEXT = nullptr;
PFN_vkQueueEndDebugUtilsLabelEXT pfvkQueueEndDebugUtilsLabelEXT = nullptr;
PFN_vkQueueInsertDebugUtilsLabelEXT pfvkQueueInsertDebugUtilsLabelEXT = nullptr;
PFN_vkSetDebugUtilsObjectNameEXT pfvkSetDebugUtilsObjectNameEXT = nullptr;
PFN_vkSetDebugUtilsObjectTagEXT pfvkSetDebugUtilsObjectTagEXT = nullptr;
#endif /* defined(VK_EXT_debug_utils) */



namespace blender {
namespace gpu {
namespace debug {
struct VKDebuggingTools {

  /*One -on -one for instance.*/
  VkInstance  instance = VK_NULL_HANDLE;
  VkDevice     device = VK_NULL_HANDLE;
  VkDebugUtilsMessengerEXT dbgMessenger = nullptr;
  Set<int32_t> dbgIgnoreMessages;
  std::mutex lists_mutex_;

  VKDebuggingTools()
  {
    instance = VK_NULL_HANDLE;
    device = VK_NULL_HANDLE;
    dbgIgnoreMessages.clear();
    dbgMessenger = nullptr;
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
}  // namespace debug
}  // namespace gpu
}  // namespace blender

namespace blender {
namespace gpu {
namespace debug {

/**
 * A function that loads a function for an instance of Vulkan.
 * Each extension must be enabled in the instance to be loaded here.
 * DLL load seems unnecessary for now.
 */
static void BlenderVulkanLoadInstance(VkInstance context)
{
#define load vkGetInstanceProcAddr

#if defined(VK_EXT_debug_utils)
  pfvkCmdBeginDebugUtilsLabelEXT = (PFN_vkCmdBeginDebugUtilsLabelEXT)load(
      context, "vkCmdBeginDebugUtilsLabelEXT");
  pfvkCmdEndDebugUtilsLabelEXT = (PFN_vkCmdEndDebugUtilsLabelEXT)load(
      context, "vkCmdEndDebugUtilsLabelEXT");
  pfvkCmdInsertDebugUtilsLabelEXT = (PFN_vkCmdInsertDebugUtilsLabelEXT)load(
      context, "vkCmdInsertDebugUtilsLabelEXT");
  pfvkCreateDebugUtilsMessengerEXT = (PFN_vkCreateDebugUtilsMessengerEXT)load(
      context, "vkCreateDebugUtilsMessengerEXT");
  pfvkDestroyDebugUtilsMessengerEXT = (PFN_vkDestroyDebugUtilsMessengerEXT)load(
      context, "vkDestroyDebugUtilsMessengerEXT");
  pfvkQueueBeginDebugUtilsLabelEXT = (PFN_vkQueueBeginDebugUtilsLabelEXT)load(
      context, "vkQueueBeginDebugUtilsLabelEXT");
  pfvkQueueEndDebugUtilsLabelEXT = (PFN_vkQueueEndDebugUtilsLabelEXT)load(
      context, "vkQueueEndDebugUtilsLabelEXT");
  pfvkQueueInsertDebugUtilsLabelEXT = (PFN_vkQueueInsertDebugUtilsLabelEXT)load(
      context, "vkQueueInsertDebugUtilsLabelEXT");
  pfvkSetDebugUtilsObjectNameEXT = (PFN_vkSetDebugUtilsObjectNameEXT)load(
      context, "vkSetDebugUtilsObjectNameEXT");
  pfvkSetDebugUtilsObjectTagEXT = (PFN_vkSetDebugUtilsObjectTagEXT)load(
      context, "vkSetDebugUtilsObjectTagEXT");
  pfvkSubmitDebugUtilsMessageEXT = (PFN_vkSubmitDebugUtilsMessageEXT)load(
      context, "vkSubmitDebugUtilsMessageEXT");
#endif /* defined(VK_EXT_debug_utils) */

#undef load
}




/*If we don't have multiple instances, VKDebuggingTools is Singleton.*/
static VKDebuggingTools tools;

#define CONSOLE_COLOR_YELLOW "\x1b[33m"
#define CONSOLE_COLOR_RED "\x1b[31m"
#define CONSOLE_COLOR_RESET "\x1b[0m"
#define CONSOLE_COLOR_FINE "\x1b[2m"

#if defined(VK_EXT_debug_utils)

VKAPI_ATTR VkResult VKAPI_CALL
vkCreateDebugUtilsMessengerEXT(VkInstance instance,
                               const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
                               const VkAllocationCallbacks *pAllocator,
                               VkDebugUtilsMessengerEXT *pMessenger)
{
  VkResult r = pfvkCreateDebugUtilsMessengerEXT(instance, pCreateInfo, pAllocator, pMessenger);
  VK_ERROR_CHECK(r, __FUNCTION__);
  return r;
};

/*Supported since Vulkan 1.0.
 * https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_debug_utils.html */
VKAPI_ATTR VkBool32 VKAPI_CALL
debugUtilsCB(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
             VkDebugUtilsMessageTypeFlagsEXT messageType,
             const VkDebugUtilsMessengerCallbackDataEXT *callbackData,
             void *userData)
{

  VKDebuggingTools *ctx = reinterpret_cast<VKDebuggingTools *>(userData);
  if (ctx->dbgIgnoreMessages.contains(callbackData->messageIdNumber)) {
    return VK_FALSE;
  }

  const bool use_color = CLG_color_support_get(&LOG);
  if (ELEM(messageSeverity,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT,
           VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)) {
    if ((LOG.type->flag & CLG_FLAG_USE) && (LOG.type->level >= CLG_SEVERITY_INFO)) {
      const char *format = use_color ? CONSOLE_COLOR_FINE "% s\n %s " CONSOLE_COLOR_RESET:" % s\n %s ";
      CLG_logf(LOG.type,
               CLG_SEVERITY_INFO,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
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

    if (clog_severity == CLG_SEVERITY_ERROR) {

      const char *format = use_color ? CONSOLE_COLOR_RED "%s\n" CONSOLE_COLOR_RESET " %s \n " : " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);

    }
    else if (LOG.type->level >= CLG_SEVERITY_WARN){
      const char *format = use_color ? CONSOLE_COLOR_YELLOW "%s\n" CONSOLE_COLOR_RESET " %s \n " : " %s\n %s ";
      CLG_logf(LOG.type,
               clog_severity,
               "",
               "",
               format,
               callbackData->pMessageIdName,
               callbackData->pMessage);
    }
  }

  return VK_TRUE;

};



static VkResult CreateDebugUtils(VkDebugUtilsMessageSeverityFlagsEXT flag, VKDebuggingTools &deb)
{

  deb.dbgIgnoreMessages.clear();
  BLI_assert(pfvkCreateDebugUtilsMessengerEXT);

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
  return pfvkCreateDebugUtilsMessengerEXT(
      deb.instance, &dbg_messenger_create_info, nullptr, &deb.dbgMessenger);
}
static VkResult DestroyDebugUtils(VKDebuggingTools &deb)
{

  BLI_assert(pfvkDestroyDebugUtilsMessengerEXT);
  pfvkDestroyDebugUtilsMessengerEXT(deb.instance, deb.dbgMessenger, nullptr);

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

bool init_vk_callbacks(void *instance)
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

  BlenderVulkanLoadInstance(tools.instance);

#if defined(VK_EXT_debug_utils)
  CreateDebug(tools);
#endif

  return true;
}

void destroy_vk_callbacks()
{

#if defined(VK_EXT_debug_utils)
  if (tools.dbgMessenger) {
    DestroyDebugUtils(tools);
  }
#endif
}

void object_vk_label(VkDevice device, VkObjectType objType, uint64_t obj, const std::string &name)
{
  if (G.debug & G_DEBUG_GPU) {
    VkDebugUtilsObjectNameInfoEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
    info.objectType = objType;
    info.objectHandle = obj;
    info.pObjectName = name.c_str();
    pfvkSetDebugUtilsObjectNameEXT(device, &info);
  }
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
template<>
void object_vk_label(VkDevice device, VkDescriptorSetLayout obj, const std::string &name)
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
  if (G.debug & G_DEBUG_GPU) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkCmdBeginDebugUtilsLabelEXT(cmd, &info);
  }
}
void setMarker(VkCommandBuffer cmd, const std::string &name)
{
  if (G.debug & G_DEBUG_GPU) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkCmdInsertDebugUtilsLabelEXT(cmd, &info);
  }
}
void popMarker(VkCommandBuffer cmd)
{
  if (G.debug & G_DEBUG_GPU) {
    pfvkCmdEndDebugUtilsLabelEXT(cmd);
  }
}
void pushMarker(VkQueue q, const std::string &name)
{
  if (G.debug & G_DEBUG_GPU) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkQueueBeginDebugUtilsLabelEXT(q, &info);
  }
}
void setMarker(VkQueue q, const std::string &name)
{
  if (G.debug & G_DEBUG_GPU) {
    VkDebugUtilsLabelEXT info = {};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    info.pLabelName = name.c_str();
    pfvkQueueInsertDebugUtilsLabelEXT(q, &info);
  }
}

void popMarker(VkQueue q)
{
  if (G.debug & G_DEBUG_GPU) {
    pfvkQueueEndDebugUtilsLabelEXT(q);
  }
}

void raise_vk_error(const char *info){

  static VkDebugUtilsMessengerCallbackDataEXT cbdata;
  cbdata.sType  = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CALLBACK_DATA_EXT;
  cbdata.pNext = 0;
  cbdata.messageIdNumber = 100;
  cbdata.pMessageIdName = "Raise";
  cbdata.objectCount = 0;
  cbdata.flags = 0;
  cbdata.pObjects = nullptr;
  cbdata.pMessage = info;
  pfvkSubmitDebugUtilsMessageEXT(tools.instance,
                                 VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
                                 VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT,
                                 &cbdata);


}
void check_vk_resources(const char *info){};

}  // namespace debug
}  // namespace gpu
}  // namespace blender

/*
 * TODO:: Comment
 */
void GHOST_VulkanInstanceLoad(void *m_instance)
{
  blender::gpu::debug::init_vk_callbacks(m_instance);
};

void GHOST_VulkanInstanceUnload()
{
  blender::gpu::debug::destroy_vk_callbacks();
}




