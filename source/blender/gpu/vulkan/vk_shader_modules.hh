/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "GPU_shader.hh"

#include "vk_common.hh"

#include "shaderc/shaderc.hpp"

namespace blender::gpu {
class VKShader;
namespace shader {
class ShaderCreateInfo;
}

/**
 * Pool of shader modules.
 *
 * Responsibility of VKShaderModules:
 * - Reusing of VkShaderModule between shaders to reduce compilation and pipeline creation
 * times.
 * - Loading of precompiled spirv bytecode.
 */
class VKShaderModules {
 private:
  struct {
    double glsl_to_spirv_time = 0.0;
    double spirv_to_shader_module_time = 0.0;
  } stats_;

 public:
  bool construct(VKShader &shader,
                 const shader::ShaderCreateInfo &info,
                 Span<const char *> sources,
                 shaderc_shader_kind stage,
                 VkShaderModule *r_shader_module);
  void destruct(VkShaderModule vk_shader_module);
  void free_data();

  void debug_print() const;

 private:
  bool compile_glsl_to_spirv(VKShader &shader,
                             Span<const char *> sources,
                             shaderc_shader_kind stage,
                             Vector<uint32_t> &r_compiled_spirv);
  bool build_shader_module(const VKShader &shader,
                           Span<uint32_t> spirv_module,
                           VkShaderModule *r_shader_module);
};

}  // namespace blender::gpu
