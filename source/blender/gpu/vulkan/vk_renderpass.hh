/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once
#include "vk_attachments.hh"
#include "vk_common.hh"
#include "vk_subpass.hh"

#include <memory>

namespace blender::gpu {
#define VK_ATTACHMENT_EMPTY -1000
/*
 * Implementing Image Transitions in the `VkRenderPass`.
 * Of course, there are various transitions that can be considered, but with the following two
 * transition types, the images used for most renders can be used without being barrierd in the
 * middle.
 *
 * type0:  `VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL` to `VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL`
 * type1:  `VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL` to `VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL`
 *
 */
namespace vk_renderpass {
const VkRenderPassCreateInfo2 create_info_default = {VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO_2,
                                                     VK_NULL_HANDLE,
                                                     0,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr,
                                                     0,
                                                     nullptr};
}
namespace vk_subpass {
const VkSubpassDescription2 descriptions_default = {
    VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_2, VK_NULL_HANDLE, 0, VK_PIPELINE_BIND_POINT_GRAPHICS};
}

/**
 * Pipeline can be a compute pipeline or a graphic pipeline.
 *
 * Compute pipelines can be constructed early on, but graphics
 * pipelines depends on the actual GPU state/context.
 *
 * - TODO: we should sanitize the interface. There we can also
 *   use late construction for compute pipelines.
 */
#ifdef VK_STAT_PIPELINE_CACHE
class Base64 {
 public:
  template<class T> static std::string Encode(const std::vector<T> &data)
  {
    return Encode((uint8_t *)data.data(), data.size() * sizeof(T));
  }
  template<class T> static std::string Encode(T data, size_t in_len)
  {

    static constexpr char sEncodingTable[] = {
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
        'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
        'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
        'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};

    size_t out_len = 4 * ((in_len + 2) / 3);
    std::string ret(out_len, '\0');
    size_t i = 0;
    char *p = const_cast<char *>(ret.c_str());

    if (in_len > 1) {
      for (i = 0; i < in_len - 2; i += 3) {
        *p++ = sEncodingTable[(data[i] >> 2) & 0x3F];
        *p++ = sEncodingTable[((data[i] & 0x3) << 4) | ((int)(data[i + 1] & 0xF0) >> 4)];
        *p++ = sEncodingTable[((data[i + 1] & 0xF) << 2) | ((int)(data[i + 2] & 0xC0) >> 6)];
        *p++ = sEncodingTable[data[i + 2] & 0x3F];
      }
    }

    if (i < in_len) {
      *p++ = sEncodingTable[(data[i] >> 2) & 0x3F];
      if (i == (in_len - 1)) {
        *p++ = sEncodingTable[((data[i] & 0x3) << 4)];
        *p++ = '=';
      }
      else {
        *p++ = sEncodingTable[((data[i] & 0x3) << 4) | ((int)(data[i + 1] & 0xF0) >> 4)];
        *p++ = sEncodingTable[((data[i + 1] & 0xF) << 2)];
      }
      *p++ = '=';
    }

    return ret;
  }
};

template<class T> std::string encode_struct(T *data, int width)
{
  size_t size = sizeof(T) * width;
  if (size <= 0)
    return "";
  Base64 b64;
  return b64.Encode((uint8_t *)(data), size);
};

#  define VK_PIPELINE_CACHE_MAX 16
#  define VK_PIPELINE_VAO_BYTES_SIZE 128
#endif

class VKRenderPass {
 private:
  bool imageless_ = false;
  bool is_clear_pass_ = false;
  bool dirty_ = false;
  VkRenderPass vk_render_pass_ = VK_NULL_HANDLE;
#ifdef VK_PIPELINE_REFACTOR
  VkPipeline vk_pipelines[GPU_PRIM_NONE][VK_PIPELINE_CACHE_MAX];
  struct cache_info {
    uint64_t state;
    uint64_t v_module;
    uint64_t g_module;
    uint64_t f_module;
    char vao_info[VK_PIPELINE_VAO_BYTES_SIZE];
  };
  cache_info pipeline_cache_by_prims[GPU_PRIM_NONE][VK_PIPELINE_CACHE_MAX];
  int pipeline_cache_nums[GPU_PRIM_NONE];
#endif
  eRenderpassType render_pass_enum_ = eRenderpassType::Any;
  VkRenderPassCreateInfo2 vk_create_info_[2] = {vk_renderpass::create_info_default,
                                                vk_renderpass::create_info_default};
  uint8_t info_id_ = 0b00;
  const uint8_t info_id_counter() const
  {
    return (info_id_ + 1) % 2;
  }

  /** There is a separate classification issue with regard to multi-layered rendering. **/
  int multiview_layers_ = 1;
  std::array<VkSubpassDescription2, 2> subpass_ = {vk_subpass::descriptions_default,
                                                   vk_subpass::descriptions_default};

  VKAttachments attachments_;
  Vector<int> subpass_multi_attachments;
  Vector<int> subpass_input_orders_[GPU_TEX_MAX_SUBPASS];

 public:
  VKRenderPass();
  ~VKRenderPass()
  {
    free();
  };
  void create();
  void free();
  bool ensure();
  void ensure_subpass_multiple(VKFrameBuffer &frame_buffer);
  void cache_init();
  void dependency_static_set(bool use_depth);
  VkSubpassDependency2 dependency_get(int srcpass,
                                      int dstpass,
                                      SubpassTransitionPattern transition_pattern,
                                      VkImageLayout dst_layout);
  void imageless_pass_set();
  void multiview_set();
#ifdef VK_PIPELINE_REFACTOR
  void pipeline_free(VKDevice &device);
  VkPipeline has_pipeline_cache(const GPUPrimType prim_type,
                                const GPUState state,
                                VkShaderModule v_module,
                                VkShaderModule g_module,
                                VkShaderModule f_module,
                                const VkPipelineVertexInputStateCreateInfo &info,
                                int type);

  VkPipeline has_pipeline_cache(const GPUPrimType prim_type,
                                const GPUState state,
                                VkShaderModule v_module,
                                VkShaderModule g_module,
                                VkShaderModule f_module,
                                int type);

  VkPipeline has_pipeline_cache(const GPUPrimType prim_type, const GPUState state, int type);

  void set_pipeline(VkPipeline pipeline,
                    const GPUPrimType prim_type,
                    const GPUState state,
                    int type);

  void set_pipeline(VkPipeline pipeline,
                    const GPUPrimType prim_type,
                    VkShaderModule v_module,
                    VkShaderModule g_module,
                    VkShaderModule f_module,
                    const GPUState state,
                    int type);

  void set_pipeline(VkPipeline pipeline,
                    const GPUPrimType prim_type,
                    VkShaderModule v_module,
                    VkShaderModule g_module,
                    VkShaderModule f_module,
                    const GPUState state,
                    const VkPipelineVertexInputStateCreateInfo &info,
                    int type);
#endif
  friend class VKFrameBuffer;
};

}  // namespace blender::gpu
