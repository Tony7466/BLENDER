/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "BLI_string_utils.hh"
#include "BLI_time.h"

#include "gpu_shader_create_info.hh"

#include "vk_backend.hh"
#include "vk_memory.hh"
#include "vk_shader.hh"
#include "vk_shader_log.hh"
#include "vk_shader_modules.hh"

#include <sstream>

namespace blender::gpu {

bool VKShaderModules::construct(VKShader &shader,
                                const shader::ShaderCreateInfo & /*info*/,
                                Span<const char *> sources,
                                shaderc_shader_kind stage,
                                VkShaderModule *r_shader_module)
{
  BLI_assert_msg(ELEM(stage,
                      shaderc_vertex_shader,
                      shaderc_geometry_shader,
                      shaderc_fragment_shader,
                      shaderc_compute_shader),
                 "Only forced ShaderC shader kinds are supported.");
  Vector<uint32_t> spirv_module;
  if (!compile_glsl_to_spirv(shader, sources, stage, spirv_module)) {
    r_shader_module = VK_NULL_HANDLE;
    return false;
  }
  return build_shader_module(shader, spirv_module, r_shader_module);
}

void VKShaderModules::destruct(VkShaderModule vk_shader_module)
{
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  vkDestroyShaderModule(device.device_get(), vk_shader_module, vk_allocation_callbacks);
}

void VKShaderModules::free_data()
{
  debug_print();
}

/* -------------------------------------------------------------------- */
/** \name Frontend compilation (GLSL -> SpirV)
 *
 * \{ */

static const std::string to_stage_name(shaderc_shader_kind stage)
{
  switch (stage) {
    case shaderc_vertex_shader:
      return std::string("vertex");
    case shaderc_geometry_shader:
      return std::string("geometry");
    case shaderc_fragment_shader:
      return std::string("fragment");
    case shaderc_compute_shader:
      return std::string("compute");

    default:
      BLI_assert_msg(false, "Do not know how to convert shaderc_shader_kind to stage name.");
      break;
  }
  return std::string("unknown stage");
}

static std::string combine_sources(Span<const char *> sources)
{
  char *sources_combined = BLI_string_join_arrayN((const char **)sources.data(), sources.size());
  std::string result(sources_combined);
  MEM_freeN(sources_combined);
  return result;
}

bool VKShaderModules::compile_glsl_to_spirv(VKShader &shader,
                                            Span<const char *> sources,
                                            shaderc_shader_kind stage,
                                            Vector<uint32_t> &r_compiled_spirv)
{
  const double start_time = BLI_time_now_seconds();
  std::string combined_sources = combine_sources(sources);
  VKBackend &backend = VKBackend::get();
  shaderc::Compiler &compiler = backend.get_shaderc_compiler();
  shaderc::CompileOptions options;
  options.SetOptimizationLevel(shaderc_optimization_level_performance);
  options.SetTargetEnvironment(shaderc_target_env_vulkan, shaderc_env_version_vulkan_1_2);
  if (G.debug & G_DEBUG_GPU_RENDERDOC) {
    options.SetOptimizationLevel(shaderc_optimization_level_zero);
    options.SetGenerateDebugInfo();
  }

  shaderc::SpvCompilationResult module = compiler.CompileGlslToSpv(
      combined_sources, stage, shader.name, options);
  if (module.GetNumErrors() != 0 || module.GetNumWarnings() != 0) {
    std::string log = module.GetErrorMessage();
    Vector<char> logcstr(log.c_str(), log.c_str() + log.size() + 1);

    VKLogParser parser;
    shader.print_log(sources,
                     logcstr.data(),
                     to_stage_name(stage).c_str(),
                     module.GetCompilationStatus() != shaderc_compilation_status_success,
                     &parser);
  }
  stats_.glsl_to_spirv_time += BLI_time_now_seconds() - start_time;
  r_compiled_spirv.clear();
  if (module.GetCompilationStatus() != shaderc_compilation_status_success) {
    return false;
  }

  r_compiled_spirv.extend(module.cbegin(), module.cend());
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Frontend compilation (SpirV -> ShaderModule)
 *
 * \{ */
bool VKShaderModules::build_shader_module(const VKShader &shader,
                                          Span<uint32_t> spirv_module,
                                          VkShaderModule *r_shader_module)
{
  const double start_time = BLI_time_now_seconds();
  VK_ALLOCATION_CALLBACKS;

  VkShaderModuleCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  create_info.codeSize = spirv_module.size() * sizeof(uint32_t);
  create_info.pCode = spirv_module.data();

  const VKDevice &device = VKBackend::get().device_get();
  VkResult result = vkCreateShaderModule(
      device.device_get(), &create_info, vk_allocation_callbacks, r_shader_module);
  stats_.spirv_to_shader_module_time += BLI_time_now_seconds() - start_time;
  if (result == VK_SUCCESS) {
    debug::object_label(*r_shader_module, shader.name);
    return true;
  }
  else {
    *r_shader_module = VK_NULL_HANDLE;
    return false;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Debugging and statistics
 *
 * \{ */

void VKShaderModules::debug_print() const
{
  std::stringstream ss;
  ss << "VKShaderModules(glsl_spirv_time=" << stats_.glsl_to_spirv_time << "s"
     << ", spirv_shader_module_time" << stats_.spirv_to_shader_module_time << "s)\n";
  std::cout << ss.str();
}

/** \} */

}  // namespace blender::gpu
