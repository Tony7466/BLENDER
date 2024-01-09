/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "MEM_guardedalloc.h"

#include <epoxy/gl.h>

#include "BLI_map.hh"

#include "gpu_shader_create_info.hh"
#include "gpu_shader_private.hh"

namespace blender {
namespace gpu {

/**
 * Shaders that uses specialization constants must keep track of the sources in order to rebuild
 * shader stages.
 *
 * Some sources are shared and won't be copied. For example for dependencies. In this case we
 * would only store the source_ref.
 *
 * Other sources would be stored in the #source attribute. #source_ref
 * would still be updated. These would include shader create info sources.
 */
struct GLSource {
  std::string source;
  StringRefNull source_ref;

  GLSource(const char *other_source);
};
class GLSources : public Vector<GLSource> {
 public:
  GLSources &operator=(Span<const char *> other);
};

/**
 * Implementation of shader compilation and uniforms handling using OpenGL.
 */
class GLShader : public Shader {
  friend shader::ShaderCreateInfo;
  friend shader::StageInterfaceInfo;

 private:
  struct SpecializationProgram {
    /** Handle for full program (links shader stages below). */
    GLuint shader_program = 0;
    /** Individual shader stages. */
    GLuint vert_shader = 0;
    GLuint geom_shader = 0;
    GLuint frag_shader = 0;
    GLuint compute_shader = 0;
    ~SpecializationProgram();
  };

  struct SpecializationPrograms {
    using Key = Vector<shader::ShaderCreateInfo::SpecializationConstant::Value>;

   private:
    Map<Key, SpecializationProgram> entries;

   public:
    /**
     * Points to the active specialization program. Use `ensure_active` or `ensure_any_active` to
     * set this attribute.
     */
    SpecializationProgram *active = nullptr;

    /**
     * When the shader uses Specialization Constants these attribute contains the sources for
     * rebuild shader stages. When Specialization Constants aren't used they are kept empty to
     * reduce memory requirements.
     */
    GLSources vertex_sources;
    GLSources geometry_sources;
    GLSources fragment_sources;
    GLSources compute_sources;

   public:
    /**
     * Ensure that the active specialization program #active points to the specialization
     * constants of the current values inside `Shader::constants.values`
     */
    void ensure_active();

    /**
     * Ensure that there is an active specialization program. This method only sets an
     * specialization program when there is no active program set yet.
     *
     * This is useful for functionality that doesn't require the correct specialization yet, but
     * checks some shared property of this shader.
     */
    void ensure_any_active();

    /**
     * Make sure that the active is filled. It might still point to an instance that isn't fully
     * compiled.
     */
    void ensure_program_created(const GLShader &shader);
  };
  SpecializationPrograms programs_;
  void update_program_and_sources(GLSources &stage_sources, MutableSpan<const char *> sources);

  /** True if any shader failed to compile. */
  bool compilation_failed_ = false;

  eGPUShaderTFBType transform_feedback_type_ = GPU_SHADER_TFB_NONE;

 public:
  GLShader(const char *name);
  ~GLShader();

  /** Return true on success. */
  void vertex_shader_from_glsl(MutableSpan<const char *> sources) override;
  void geometry_shader_from_glsl(MutableSpan<const char *> sources) override;
  void fragment_shader_from_glsl(MutableSpan<const char *> sources) override;
  void compute_shader_from_glsl(MutableSpan<const char *> sources) override;
  bool finalize(const shader::ShaderCreateInfo *info = nullptr) override;
  void warm_cache(int /*limit*/) override{};

  std::string resources_declare(const shader::ShaderCreateInfo &info) const override;
  std::string constants_declare() const;
  std::string vertex_interface_declare(const shader::ShaderCreateInfo &info) const override;
  std::string fragment_interface_declare(const shader::ShaderCreateInfo &info) const override;
  std::string geometry_interface_declare(const shader::ShaderCreateInfo &info) const override;
  std::string geometry_layout_declare(const shader::ShaderCreateInfo &info) const override;
  std::string compute_layout_declare(const shader::ShaderCreateInfo &info) const override;

  /** Should be called before linking. */
  void transform_feedback_names_set(Span<const char *> name_list,
                                    eGPUShaderTFBType geom_type) override;
  bool transform_feedback_enable(GPUVertBuf *buf) override;
  void transform_feedback_disable() override;

  void bind() override;
  void unbind() override;

  void uniform_float(int location, int comp_len, int array_size, const float *data) override;
  void uniform_int(int location, int comp_len, int array_size, const int *data) override;

  /* Unused: SSBO vertex fetch draw parameters. */
  bool get_uses_ssbo_vertex_fetch() const override
  {
    return false;
  }
  int get_ssbo_vertex_fetch_output_num_verts() const override
  {
    return 0;
  }

  /** DEPRECATED: Kept only because of BGL API. */
  int program_handle_get() const override;

  bool is_compute() const
  {
    if (!programs_.vertex_sources.is_empty()) {
      return false;
    }
    if (!programs_.compute_sources.is_empty()) {
      return true;
    }
    return programs_.active->compute_shader;
  }

 private:
  const char *glsl_patch_get(GLenum gl_stage);

  /** Create, compile and attach the shader stage to the shader program. */
  GLuint create_shader_stage(GLenum gl_stage, MutableSpan<const char *> sources);

  /**
   * \brief features available on newer implementation such as native barycentric coordinates
   * and layered rendering, necessitate a geometry shader to work on older hardware.
   */
  std::string workaround_geometry_shader_source_create(const shader::ShaderCreateInfo &info);

  bool do_geometry_shader_injection(const shader::ShaderCreateInfo *info);

  MEM_CXX_CLASS_ALLOC_FUNCS("GLShader");
};

class GLLogParser : public GPULogParser {
 public:
  const char *parse_line(const char *source_combined,
                         const char *log_line,
                         GPULogItem &log_item) override;

 protected:
  const char *skip_severity_prefix(const char *log_line, GPULogItem &log_item);
  const char *skip_severity_keyword(const char *log_line, GPULogItem &log_item);

  MEM_CXX_CLASS_ALLOC_FUNCS("GLLogParser");
};

}  // namespace gpu
}  // namespace blender
