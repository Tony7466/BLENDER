/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_backend.hh"

#include "BLI_string_ref.hh"

#include "gpu_shader_create_info.hh"

namespace blender::gpu {
class VKShader;

/**
 * Class to keep temporarily track of the sources required to compile a shader so the shader can be
 * patched later on when the ShaderCreateInfo has been constructed.
 *
 * NOTE: we might want to change the gpu intern as a similar construction is done for metal as
 * well, making OpenGL the exception.
 */
class VKShaderBuilder {
  VKShader &shader_;
  Vector<std::string> vertex_glsl_;
  Vector<std::string> geometry_glsl_;
  Vector<std::string> fragment_glsl_;
  Vector<std::string> compute_glsl_;

 public:
  VKShaderBuilder(VKShader &shader) : shader_(shader) {}
  void set_vertex_glsl(Span<const char *> glsl);
  void set_geometry_glsl(Span<const char *> glsl);
  void set_fragment_glsl(Span<const char *> glsl);
  void set_compute_glsl(Span<const char *> glsl);

  void patch_glsl(const shader::ShaderCreateInfo &info);

  bool build_vertex_module(VkShaderModule *r_module);
  bool build_fragment_module(VkShaderModule *r_module);
  bool build_geometry_module(VkShaderModule *r_module);
  bool build_compute_module(VkShaderModule *r_module);

 private:
  bool build_module(Span<std::string> sources,
                    shaderc_shader_kind stage,
                    VkShaderModule *r_module);
};

}  // namespace blender::gpu
