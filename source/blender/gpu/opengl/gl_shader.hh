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
template<>
struct DefaultHash<Vector<gpu::shader::ShaderCreateInfo::SpecializationConstant::Value>> {
  uint64_t operator()(
      const Vector<gpu::shader::ShaderCreateInfo::SpecializationConstant::Value> &key) const
  {
    uint64_t hash = 0;
    for (const gpu::shader::ShaderCreateInfo::SpecializationConstant::Value &value : key) {
      hash = hash * 33 + value.u;
    }
    return hash;
  }
};

namespace gpu {

/**
 * Shaders that uses specialization constants must keep track of the sources in order to rebuild
 * shader stages.
 *
 * Some sources are shared and won't be copied. For example for dependencies. In this case we
 * would only store the source_ref.
 *
 * Other sources would be stored in the #source attribute. #source_ref
 * would still be updated.
 */
struct GLSource {
  std::string source;
  StringRefNull source_ref;

  GLSource() = default;
  GLSource(const char *other_source);
};
class GLSources : public Vector<GLSource> {
 public:
  GLSources &operator=(Span<const char *> other);
  Vector<const char *> update_constants_source(const StringRefNull constants_patch) const;

 private:
  /**
   * Index into self where the constants needs to be applied before.
   * -1 means no constants will be applied.
   */
  int32_t constants_source_index_ = -1;
};

/**
 * Implementation of shader compilation and uniforms handling using OpenGL.
 */
class GLShader : public Shader {
  friend shader::ShaderCreateInfo;
  friend shader::StageInterfaceInfo;

 private:
  struct GLProgram {
    /** Handle for program. */
    GLuint program_id = 0;
    /** Handle for individual shader stages. */
    GLuint vert_shader = 0;
    GLuint geom_shader = 0;
    GLuint frag_shader = 0;
    GLuint compute_shader = 0;
    ~GLProgram();
    void link(Shader &shader);
  };

  struct GLPrograms {
    using Key = Vector<shader::ShaderCreateInfo::SpecializationConstant::Value>;

   private:
    GLShader &shader_;
    Map<Key, GLProgram> program_cache_;

   public:
    /**
     * Points to the active program. When binding a shader the active shader is
     * setup.
     */
    GLProgram *program_active = nullptr;

    /**
     * When the shader uses Specialization Constants these attribute contains the sources for
     * rebuild shader stages. When Specialization Constants aren't used they are empty to
     * reduce memory needs.
     */
    GLSources vertex_sources;
    GLSources geometry_sources;
    GLSources fragment_sources;
    GLSources compute_sources;

   public:
    GLPrograms(GLShader &shader) : shader_(shader) {}

    /**
     * Initialize this instance.
     *
     * - Ensures that entries at least has a default GLProgram.
     * - Ensures that active is set.
     * - Active GLProgram has a shader_program (at least in creation state).
     * - Does nothing when instance was already initialized.
     */
    void init();

    /**
     * Return a GLProgram program id that reflects the current state of shader.constants.values.
     * The returned program_id is in linked state, or an error happened during linking.
     */
    GLuint program_get();
  };

  GLPrograms programs_;
  Vector<const char *> specialization_constant_names_;
  void update_program_and_sources(GLSources &stage_sources, MutableSpan<const char *> sources);

  /** True if any shader failed to compile. */
  bool compilation_failed_ = false;

  eGPUShaderTFBType transform_feedback_type_ = GPU_SHADER_TFB_NONE;

 public:
  GLShader(const char *name);
  ~GLShader();

  void init(const shader::ShaderCreateInfo &info) override;

  /** Return true on success. */
  void vertex_shader_from_glsl(MutableSpan<const char *> sources) override;
  void geometry_shader_from_glsl(MutableSpan<const char *> sources) override;
  void fragment_shader_from_glsl(MutableSpan<const char *> sources) override;
  void compute_shader_from_glsl(MutableSpan<const char *> sources) override;
  bool finalize(const shader::ShaderCreateInfo *info = nullptr) override;
  void warm_cache(int /*limit*/) override{};

  std::string resources_declare(const shader::ShaderCreateInfo &info) const override;
  std::string constants_declare(const shader::ShaderCreateInfo &info) const override;
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
    return programs_.program_active->compute_shader;
  }

 private:
  const char *glsl_patch_get(GLenum gl_stage);

  /** Create, compile and attach the shader stage to the shader program. */
  GLuint create_shader_stage(GLenum gl_stage,
                             MutableSpan<const char *> sources,
                             const GLSources &gl_sources);

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
