/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 *
 * Debug features of Vulkan.
 */


#include  "BLI_set.hh"
#include  "BKE_global.h"

#include  "GPU_debug.h"
#include  "GPU_platform.h"

#include  "CLG_log.h"

#include  "vk_common.hh"
#include  "vk_debug.hh"
#include  "vk_context.hh"

#include  "GHOST_C-api.h"


#if _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  include <windows.h>

HMODULE  VulkanLibary;
#  define VulkanLoadLibrary LoadLibraryA
#  define VulkanLoadProcAddr GetProcAddress

#  define VULKANLIB "vulkan-1.dll"
#endif

#include <sstream>


static CLG_LogRef LOG = {"gpu.debug.vulkan"};


PFN_vkGetInstanceProcAddr pfvkGetInstanceProcAddr;

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



VkResult initVulkanDLL()
  {

    HMODULE module = VulkanLoadLibrary(VULKANLIB);

    if (!module) {
      return VK_ERROR_INITIALIZATION_FAILED;
    }

    pfvkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)(void (*)(void))VulkanLoadProcAddr(module, "vkGetInstanceProcAddr");

    return VK_SUCCESS;

}
static void BlenderVulkanLoadInstance(VkInstance context)
{
#define load vkGetInstanceProcAddr

#if defined(VK_EXT_debug_utils) 
  pfvkCmdBeginDebugUtilsLabelEXT = (PFN_vkCmdBeginDebugUtilsLabelEXT)load(
      context, "vkCmdBeginDebugUtilsLabelEXT");
  pfvkCmdEndDebugUtilsLabelEXT = (PFN_vkCmdEndDebugUtilsLabelEXT)load(context,
                                                                    "vkCmdEndDebugUtilsLabelEXT");
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





/*Supported since Vulkan 1.0. https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_debug_utils.html */
VKAPI_ATTR VkBool32 VKAPI_CALL debugUtilsCB(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
             VkDebugUtilsMessageTypeFlagsEXT messageType,
             const VkDebugUtilsMessengerCallbackDataEXT *callbackData,
             void *userData)
{

  VKDebuggingTools *ctx = reinterpret_cast<VKDebuggingTools *>(userData);

  if (ctx->dbgIgnoreMessages.contains(callbackData->messageIdNumber)) {
    return VK_FALSE;
  }

  if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
    printf(  "vk_debug.cc::VERBOSE:: %s\n  %s\n",
                 callbackData->pMessageIdName,
                 callbackData->pMessage);
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
    printf("vk_debug.cc::INFO: %s\n  %s\n", callbackData->pMessageIdName, callbackData->pMessage);
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    printf("vk_debug.cc::WARNING: %s\n  %s\n", callbackData->pMessageIdName, callbackData->pMessage);
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    printf("vk_debug.cc::ERROR: %s\n  %s\n", callbackData->pMessageIdName, callbackData->pMessage);
  }
  else if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT) {
    printf("vk_debug.cc::GENERAL: %s\n  %s\n", callbackData->pMessageIdName, callbackData->pMessage);
  }
  else {
    printf("vk_debug.cc::%s\n  %s\n", callbackData->pMessageIdName, callbackData->pMessage);
  }


  return VK_FALSE;
}

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

  BLI_assert( initVulkanDLL() == VK_SUCCESS);

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

void init_vk_debug_layer(){};
void raise_vk_error(const char *info){};
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

const char *GHOST_VulkanErrorAsString(int64_t result)
{
#define FORMAT_ERROR(X) \
  case X: { \
    return "" #X; \
  }

  switch ((VkResult)result) {
    FORMAT_ERROR(VK_NOT_READY);
    FORMAT_ERROR(VK_TIMEOUT);
    FORMAT_ERROR(VK_EVENT_SET);
    FORMAT_ERROR(VK_EVENT_RESET);
    FORMAT_ERROR(VK_INCOMPLETE);
    FORMAT_ERROR(VK_ERROR_OUT_OF_HOST_MEMORY);
    FORMAT_ERROR(VK_ERROR_OUT_OF_DEVICE_MEMORY);
    FORMAT_ERROR(VK_ERROR_INITIALIZATION_FAILED);
    FORMAT_ERROR(VK_ERROR_DEVICE_LOST);
    FORMAT_ERROR(VK_ERROR_MEMORY_MAP_FAILED);
    FORMAT_ERROR(VK_ERROR_LAYER_NOT_PRESENT);
    FORMAT_ERROR(VK_ERROR_EXTENSION_NOT_PRESENT);
    FORMAT_ERROR(VK_ERROR_FEATURE_NOT_PRESENT);
    FORMAT_ERROR(VK_ERROR_INCOMPATIBLE_DRIVER);
    FORMAT_ERROR(VK_ERROR_TOO_MANY_OBJECTS);
    FORMAT_ERROR(VK_ERROR_FORMAT_NOT_SUPPORTED);
    FORMAT_ERROR(VK_ERROR_FRAGMENTED_POOL);
    FORMAT_ERROR(VK_ERROR_UNKNOWN);
    FORMAT_ERROR(VK_ERROR_OUT_OF_POOL_MEMORY);
    FORMAT_ERROR(VK_ERROR_INVALID_EXTERNAL_HANDLE);
    FORMAT_ERROR(VK_ERROR_FRAGMENTATION);
    FORMAT_ERROR(VK_ERROR_INVALID_OPAQUE_CAPTURE_ADDRESS);
    FORMAT_ERROR(VK_ERROR_SURFACE_LOST_KHR);
    FORMAT_ERROR(VK_ERROR_NATIVE_WINDOW_IN_USE_KHR);
    FORMAT_ERROR(VK_SUBOPTIMAL_KHR);
    FORMAT_ERROR(VK_ERROR_OUT_OF_DATE_KHR);
    FORMAT_ERROR(VK_ERROR_INCOMPATIBLE_DISPLAY_KHR);
    FORMAT_ERROR(VK_ERROR_VALIDATION_FAILED_EXT);
    FORMAT_ERROR(VK_ERROR_INVALID_SHADER_NV);
    FORMAT_ERROR(VK_ERROR_INVALID_DRM_FORMAT_MODIFIER_PLANE_LAYOUT_EXT);
    FORMAT_ERROR(VK_ERROR_NOT_PERMITTED_EXT);
    FORMAT_ERROR(VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT);
    FORMAT_ERROR(VK_THREAD_IDLE_KHR);
    FORMAT_ERROR(VK_THREAD_DONE_KHR);
    FORMAT_ERROR(VK_OPERATION_DEFERRED_KHR);
    FORMAT_ERROR(VK_OPERATION_NOT_DEFERRED_KHR);
    FORMAT_ERROR(VK_PIPELINE_COMPILE_REQUIRED_EXT);
    default:
      return "Unknown Error";
  }
}

void GHOST_VulkanInstanceUnload()
{
  blender::gpu::debug::destroy_vk_callbacks();
}



#if 0
  void DebugMaster::ignoreDebugMessage(int32_t msgID)
{
  dbgIgnoreMessages.insert(msgID);
}
#endif
