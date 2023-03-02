/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "GPU_shader.h"

#include "gpu_shader_create_info.hh"

#include "../select/select_instance.hh"

namespace blender::draw::overlay {

/**
 * Shader module. Shared between instances.
 */
template<typename SelectEngineT, bool ClippingEnabled = false> class ShaderModule {
 private:
  /** Shared shader module across all engine instances. */
  static ShaderModule *g_shader_module;

  /** TODO: Support clipping. This global state should be set by the overlay::Instance and switch
   * to the shader variations that use clipping. */
  // bool clipping_enabled = false;

  class Shader {
   protected:
    GPUShader *shader_ = nullptr;

   public:
    Shader() = default;

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
   * Special class for any shader that needs to have clipped and selection variations.
   */
  class ShaderGeometry : public Shader {
   public:
    ShaderGeometry(const char *create_info_name)
    {
      /* TODO: This is what it should be like with all variations defined with create infos. */
      // std::string create_info_name = base_create_info;
      // create_info_name += SelectEngineT::shader_suffix;
      // create_info_name += ClippingEnabled ? "_clipped" : "";
      // this->shader_ = GPU_shader_create_from_info_name(create_info_name.c_str());

      /* WORKAROUND: ... but for now, we have to patch the create info used by the old engine. */
      gpu::shader::ShaderCreateInfo info =
          *reinterpret_cast<const gpu::shader::ShaderCreateInfo *>(
              GPU_shader_create_info_get(create_info_name));

      info.define(SelectEngineT::shader_define);

      this->shader_ = GPU_shader_create_from_info(
          reinterpret_cast<const GPUShaderCreateInfo *>(&info));
    }

    /* WORKAROUND: Create a shader using a patch function to patch the create info. */
    ShaderGeometry(const char *create_info_name,
                   std::function<void(gpu::shader::ShaderCreateInfo &info)> patch)
    {
      gpu::shader::ShaderCreateInfo info =
          *reinterpret_cast<const gpu::shader::ShaderCreateInfo *>(
              GPU_shader_create_info_get(create_info_name));

      patch(info);
      SelectEngineT::SelectShader::patch(info);

      this->shader_ = GPU_shader_create_from_info(
          reinterpret_cast<const GPUShaderCreateInfo *>(&info));
    }
  };

 public:
  /** ShaderGeometry */

  ShaderGeometry armature_sphere_outline = {
      "overlay_armature_sphere_outline", [](gpu::shader::ShaderCreateInfo &info) {
        using namespace blender::gpu::shader;
        info.storage_buf(0, Qualifier::READ, "mat4", "data_buf[]");
        info.define("inst_obmat", "data_buf[gl_InstanceID]");
        info.vertex_inputs_.pop_last();
      }};

  ShaderGeometry depth_mesh = {"overlay_depth_only", [](gpu::shader::ShaderCreateInfo &info) {
                                 using namespace blender::gpu::shader;
                                 info.additional_infos_.clear();
                                 info.additional_info(
                                     "draw_view", "draw_modelmat_new", "draw_resource_handle_new");
                               }};

  ShaderGeometry extra_shape = {
      "overlay_extra", [](gpu::shader::ShaderCreateInfo &info) {
        using namespace blender::gpu::shader;
        info.storage_buf(0, Qualifier::READ, "ExtraInstanceData", "data_buf[]");
        info.define("color", "data_buf[gl_InstanceID].color_");
        info.define("inst_obmat", "data_buf[gl_InstanceID].object_to_world_");
        info.vertex_inputs_.pop_last();
        info.vertex_inputs_.pop_last();
      }};

  /** Shader */

  Shader grid = {"overlay_grid"};

  Shader background_fill = {"overlay_background"};
  Shader background_clip_bound = {"overlay_clipbound"};

  /** Module */

  /** Only to be used by Instance constructor. */
  static ShaderModule &module_get()
  {
    if (g_shader_module == nullptr) {
      /* TODO(@fclem) thread-safety. */
      g_shader_module = new ShaderModule();
    }
    return *g_shader_module;
  }

  static void module_free()
  {
    if (g_shader_module != nullptr) {
      /* TODO(@fclem) thread-safety. */
      delete g_shader_module;
      g_shader_module = nullptr;
    }
  }
};

void shader_module_free();

}  // namespace blender::draw::overlay
