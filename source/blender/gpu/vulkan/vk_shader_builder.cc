/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_shader_builder.hh"
#include "vk_memory.hh"
#include "vk_shader.hh"
#include "vk_shader_log.hh"

#include "BLI_string_utils.h"

namespace blender::gpu {

/* -------------------------------------------------------------------- */
/** \name Setters
 *
 * \{ */

void VKShaderBuilder::set_vertex_glsl(Span<const char *> glsl)
{
  vertex_glsl_ = glsl;
}

void VKShaderBuilder::set_geometry_glsl(Span<const char *> glsl)
{
  geometry_glsl_ = glsl;
}

void VKShaderBuilder::set_fragment_glsl(Span<const char *> glsl)
{
  fragment_glsl_ = glsl;
}

void VKShaderBuilder::set_compute_glsl(Span<const char *> glsl)
{
  compute_glsl_ = glsl;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name GLSL Patching
 *
 * \{ */

struct PatchRule {
  enum class Stage {
    VERTEX,
    GEOMETRY,
    FRAGMENT,
    COMPUTE,
  };
  Stage stage;

  std::string find_prefix;
  std::string replace_prefix;
  std::string find_suffix;
  std::string replace_suffix;
};

static void build_instance_interfaces_rules(const shader::ShaderCreateInfo &info,
                                            Vector<PatchRule> &rules)
{
  for (const shader::StageInterfaceInfo *iface : info.vertex_out_interfaces_) {
    if (iface->instance_name.is_empty()) {
      continue;
    }

    for (const shader::StageInterfaceInfo::InOut &inout : iface->inouts) {
      /* Vertex Out */
      PatchRule rule = {};
      rule.stage = PatchRule::Stage::VERTEX;

      std::stringstream find_prefix;
      find_prefix << iface->instance_name << "." << inout.name;
      rule.find_prefix = find_prefix.str();

      std::stringstream replace_prefix;
      replace_prefix << iface->instance_name;
      switch (inout.interp) {
        case shader::Interpolation::FLAT:
          replace_prefix << "_flat";
          break;
        case shader::Interpolation::SMOOTH:
          replace_prefix << "_smooth";
          break;
        case shader::Interpolation::NO_PERSPECTIVE:
          replace_prefix << "_noperspective";
          break;
      }
      replace_prefix << "." << inout.name;
      rule.replace_prefix = replace_prefix.str();

      rules.append(rule);
    }
  }
}

static std::string apply_rule(const PatchRule &rule, std::string &glsl)
{
  std::stringstream ss;
  std::string::size_type pos = 0;
  std::string::size_type found_index;

  while ((found_index = glsl.find(rule.find_prefix, pos)) != std::string::npos) {
    ss << glsl.substr(pos, found_index - pos);
    ss << rule.replace_prefix;
    pos = found_index + rule.find_prefix.size();
  }

  ss << glsl.substr(pos);
  return ss.str();
}

static void apply_rules(const Vector<PatchRule> &rules,
                        Vector<std::string> &sources,
                        PatchRule::Stage stage)
{
  if (sources.is_empty()) {
    return;
  }

  for (int source_index : IndexRange(sources.size())) {
    std::string source = sources[source_index];
    for (const PatchRule &rule : rules) {
      if (rule.stage != stage) {
        continue;
      }
      source = apply_rule(rule, source);
    }
    if (source != sources[source_index]) {
      std::cout << "ORIGINAL:\n" << sources[source_index] << "PATCHED\n" << source << "\n";
    }
    sources[source_index] = source;
  }
}

void VKShaderBuilder::patch_glsl(const shader::ShaderCreateInfo &info)
{
  Vector<PatchRule> rules;
  build_instance_interfaces_rules(info, rules);
  if (rules.is_empty()) {
    return;
  }

  apply_rules(rules, vertex_glsl_, PatchRule::Stage::VERTEX);
  apply_rules(rules, geometry_glsl_, PatchRule::Stage::GEOMETRY);
  apply_rules(rules, fragment_glsl_, PatchRule::Stage::FRAGMENT);
  apply_rules(rules, compute_glsl_, PatchRule::Stage::COMPUTE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Building Modules
 *
 * \{ */

bool VKShaderBuilder::build_vertex_module(VkShaderModule *r_module)
{
  return build_module(vertex_glsl_, shaderc_vertex_shader, r_module);
}

bool VKShaderBuilder::build_geometry_module(VkShaderModule *r_module)
{
  return build_module(geometry_glsl_, shaderc_geometry_shader, r_module);
}

bool VKShaderBuilder::build_fragment_module(VkShaderModule *r_module)
{
  return build_module(fragment_glsl_, shaderc_fragment_shader, r_module);
}

bool VKShaderBuilder::build_compute_module(VkShaderModule *r_module)
{
  return build_module(compute_glsl_, shaderc_compute_shader, r_module);
}

static std::string combine_sources(Span<std::string> sources)
{
  std::stringstream ss;
  for (const std::string &source : sources) {
    ss << source;
  }
  return ss.str();
}

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

static Vector<uint32_t> compile_glsl_to_spirv(VKShader &shader,
                                              Span<std::string> sources,
                                              shaderc_shader_kind stage)
{
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
      combined_sources, stage, shader.name_get(), options);
  if (module.GetNumErrors() != 0 || module.GetNumWarnings() != 0) {
    std::cout << combined_sources;

    std::string log = module.GetErrorMessage();
    Vector<char> logcstr(log.c_str(), log.c_str() + log.size() + 1);

    Vector<const char *> sources_as_c;
    for (const std::string &source : sources) {
      sources_as_c.append(source.c_str());
    }

    VKLogParser parser;
    shader.print_log(sources_as_c,
                     logcstr.data(),
                     to_stage_name(stage).c_str(),
                     module.GetCompilationStatus() != shaderc_compilation_status_success,
                     &parser);
  }

  if (module.GetCompilationStatus() != shaderc_compilation_status_success) {
    return Vector<uint32_t>();
  }

  return Vector<uint32_t>(module.cbegin(), module.cend());
}

static bool build_shader_module(Span<uint32_t> spirv_module, VkShaderModule *r_shader_module)
{
  VK_ALLOCATION_CALLBACKS;

  VkShaderModuleCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  create_info.codeSize = spirv_module.size() * sizeof(uint32_t);
  create_info.pCode = spirv_module.data();

  const VKDevice &device = VKBackend::get().device_get();
  VkResult result = vkCreateShaderModule(
      device.device_get(), &create_info, vk_allocation_callbacks, r_shader_module);
  if (result != VK_SUCCESS) {
    *r_shader_module = VK_NULL_HANDLE;
    return false;
  }
  return true;
}

bool VKShaderBuilder::build_module(Span<std::string> sources,
                                   shaderc_shader_kind stage,
                                   VkShaderModule *r_module)
{
  if (sources.is_empty()) {
    return true;
  }

  BLI_assert_msg(ELEM(stage,
                      shaderc_vertex_shader,
                      shaderc_geometry_shader,
                      shaderc_fragment_shader,
                      shaderc_compute_shader),
                 "Only forced ShaderC shader kinds are supported.");
  Vector<uint32_t> spirv_module = compile_glsl_to_spirv(shader_, sources, stage);
  if (spirv_module.is_empty()) {
    return false;
  }
  return build_shader_module(spirv_module, r_module);
}

/** \} */

}  // namespace blender::gpu
