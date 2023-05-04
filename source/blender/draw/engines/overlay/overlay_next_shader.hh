/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "GPU_shader.h"

#include "gpu_shader_create_info.hh"

#include "../select/select_instance.hh"

namespace blender::draw::overlay {

using eSelectionType = select::eSelectionType;

class Shader {
 protected:
  GPUShader *shader_ = nullptr;

 public:
  Shader(GPUShader *shader) : shader_(shader){};

  /* Disable copies */
  Shader(const Shader &shader) = delete;

  Shader(const char *create_info_name)
      : shader_(GPU_shader_create_from_info_name(create_info_name)){};

  ~Shader()
  {
    DRW_SHADER_FREE_SAFE(shader_);
  }

  operator GPUShader *()
  {
    return shader_;
  }
};

/**
 * Shader module. Shared between instances.
 */
class ShaderModule {
 private:
  const eSelectionType selection_type_;
  /** TODO: Support clipping. This global state should be set by the overlay::Instance and switch
   * to the shader variations that use clipping. */
  const bool clipping_enabled_;

  /** Shared shader module across all engine instances. */
  static ShaderModule *g_shader_modules[2 /*Selection Instance*/][2 /*Clipping Enabled*/];

  Shader selectable_shader(const char *create_info_name)
  {
    /* TODO: This is what it should be like with all variations defined with create infos. */
    // std::string create_info_name = base_create_info;
    // create_info_name += SelectEngineT::shader_suffix;
    // create_info_name += ClippingEnabled ? "_clipped" : "";
    // this->shader_ = GPU_shader_create_from_info_name(create_info_name.c_str());

    /* WORKAROUND: ... but for now, we have to patch the create info used by the old engine. */
    gpu::shader::ShaderCreateInfo info = *reinterpret_cast<const gpu::shader::ShaderCreateInfo *>(
        GPU_shader_create_info_get(create_info_name));

    if (selection_type_ != eSelectionType::DISABLED) {
      info.define("SELECT_ENABLE");
    }

    return {GPU_shader_create_from_info(reinterpret_cast<const GPUShaderCreateInfo *>(&info))};
  }

  Shader selectable_shader(const char *create_info_name,
                           std::function<void(gpu::shader::ShaderCreateInfo &info)> patch)
  {
    gpu::shader::ShaderCreateInfo info = *reinterpret_cast<const gpu::shader::ShaderCreateInfo *>(
        GPU_shader_create_info_get(create_info_name));

    patch(info);

    if (selection_type_ != eSelectionType::DISABLED) {
      info.define("SELECT_ENABLE");
      /* Replace additional info. */
      for (StringRefNull &str : info.additional_infos_) {
        if (str == "draw_modelmat_new") {
          str = "draw_modelmat_new_with_custom_id";
        }
      }
      info.additional_info("select_id_patch");
    }

    return {GPU_shader_create_from_info(reinterpret_cast<const GPUShaderCreateInfo *>(&info))};
  }

 public:
  /** Selectable Shaders */

  ShaderModule(const eSelectionType selection_type, const bool clipping_enabled)
      : selection_type_(selection_type), clipping_enabled_(clipping_enabled){};

  Shader armature_sphere_outline = selectable_shader(
      "overlay_armature_sphere_outline", [](gpu::shader::ShaderCreateInfo &info) {
        using namespace blender::gpu::shader;
        info.storage_buf(0, Qualifier::READ, "mat4", "data_buf[]");
        info.define("inst_obmat", "data_buf[gl_InstanceID]");
        info.vertex_inputs_.pop_last();
      });

  Shader depth_mesh = selectable_shader(
      "overlay_depth_only", [](gpu::shader::ShaderCreateInfo &info) {
        using namespace blender::gpu::shader;
        info.additional_infos_.clear();
        info.additional_info("draw_view", "draw_modelmat_new", "draw_resource_handle_new");
      });

  Shader extra_shape = selectable_shader("overlay_extra", [](gpu::shader::ShaderCreateInfo &info) {
    using namespace blender::gpu::shader;
    info.storage_buf(0, Qualifier::READ, "ExtraInstanceData", "data_buf[]");
    info.define("color", "data_buf[gl_InstanceID].color_");
    info.define("inst_obmat", "data_buf[gl_InstanceID].object_to_world_");
    info.vertex_inputs_.pop_last();
    info.vertex_inputs_.pop_last();
  });

  /** Shader */

  Shader grid = {"overlay_grid"};

  Shader background_fill = {"overlay_background"};
  Shader background_clip_bound = {"overlay_clipbound"};

  /** Module */

  /** Only to be used by Instance constructor. */
  static ShaderModule &module_get(eSelectionType selection_type, bool clipping_enabled)
  {
    int selection_index = selection_type == eSelectionType::DISABLED ? 0 : 1;
    ShaderModule *&g_shader_module = g_shader_modules[selection_index][clipping_enabled];
    if (g_shader_module == nullptr) {
      /* TODO(@fclem) thread-safety. */
      g_shader_module = new ShaderModule(selection_type, clipping_enabled);
    }
    return *g_shader_module;
  }

  static void module_free()
  {
    for (int i : IndexRange(2)) {
      for (int j : IndexRange(2)) {
        if (g_shader_modules[i][j] != nullptr) {
          /* TODO(@fclem) thread-safety. */
          delete g_shader_modules[i][j];
          g_shader_modules[i][j] = nullptr;
        }
      }
    }
  }
};

void shader_module_free();

}  // namespace blender::draw::overlay
