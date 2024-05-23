/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GPU_compilation_subprocess.hh"

#ifdef WITH_OPENGL_BACKEND

#  include "BKE_appdir.hh"
#  include "BLI_fileops.hh"
#  include "BLI_hash.hh"
#  include "BLI_path_util.h"
#  include "CLG_log.h"
#  include "GHOST_C-api.h"
#  include "GPU_context.hh"
#  include "GPU_init_exit.hh"
#  include "opengl/ipc.hh"
#  include <epoxy/gl.h>
#  include <string>

namespace blender::gpu {

class SubprocessShader {
  GLuint vert_ = 0;
  GLuint frag_ = 0;
  GLuint program_ = 0;
  bool success_ = false;

 public:
  SubprocessShader(const char *vert_src, const char *frag_src)
  {
    GLint status;

    vert_ = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert_, 1, &vert_src, nullptr);
    glCompileShader(vert_);
    glGetShaderiv(vert_, GL_COMPILE_STATUS, &status);
    if (!status) {
      return;
    }

    frag_ = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag_, 1, &frag_src, nullptr);
    glCompileShader(frag_);
    glGetShaderiv(frag_, GL_COMPILE_STATUS, &status);
    if (!status) {
      return;
    }

    program_ = glCreateProgram();
    glAttachShader(program_, vert_);
    glAttachShader(program_, frag_);
    glLinkProgram(program_);
    glGetProgramiv(program_, GL_LINK_STATUS, &status);
    if (!status) {
      return;
    }

    success_ = true;
  }

  ~SubprocessShader()
  {
    glDeleteShader(vert_);
    glDeleteShader(frag_);
    glDeleteProgram(program_);
  }

  ShaderBinaryHeader *get_binary(void *memory)
  {
    ShaderBinaryHeader *bin = reinterpret_cast<ShaderBinaryHeader *>(memory);
    bin->format = 0;
    bin->size = 0;

    if (success_) {
      glGetProgramiv(program_, GL_PROGRAM_BINARY_LENGTH, &bin->size);
      if (bin->size + sizeof(ShaderBinaryHeader) < compilation_subprocess_shared_memory_size) {
        glGetProgramBinary(program_, bin->size, nullptr, &bin->format, &bin->data_start);
      }
    }

    return bin;
  }
};

/* Check if the binary is valid and can be loaded by the driver. */
static bool validate_binary(void *binary)
{
  ShaderBinaryHeader *bin = reinterpret_cast<ShaderBinaryHeader *>(binary);
  GLuint program = glCreateProgram();
  glProgramBinary(program, bin->format, &bin->data_start, bin->size);
  GLint status;
  glGetProgramiv(program, GL_LINK_STATUS, &status);
  glDeleteProgram(program);
  return status;
}

}  // namespace blender::gpu

void GPU_compilation_subprocess_run(const char *subprocess_name)
{
  using namespace blender;
  using namespace blender::gpu;

  CLG_init();

  std::string name = subprocess_name;
  SharedMemory shared_mem(name, compilation_subprocess_shared_memory_size, true);
  if (!shared_mem.get_data()) {
    return;
  }
  SharedSemaphore start_semaphore(name + "_START");
  SharedSemaphore end_semaphore(name + "_END");
  SharedSemaphore close_semaphore(name + "_CLOSE");

  GHOST_SystemHandle ghost_system = GHOST_CreateSystemBackground();
  BLI_assert(ghost_system);
  GHOST_GPUSettings gpu_settings = {0};
  gpu_settings.context_type = GHOST_kDrawingContextTypeOpenGL;
  GHOST_ContextHandle ghost_context = GHOST_CreateGPUContext(ghost_system, gpu_settings);
  if (ghost_context == nullptr) {
    GHOST_DisposeSystem(ghost_system);
    return;
  }
  GHOST_ActivateGPUContext(ghost_context);
  GPUContext *gpu_context = GPU_context_create(nullptr, ghost_context);
  GPU_init();

  BKE_tempdir_init(nullptr);
  std::string cache_dir = std::string(BKE_tempdir_base()) + "BLENDER_SHADER_CACHE" + SEP_STR;
  BLI_dir_create_recursive(cache_dir.c_str());

  while (true) {
    start_semaphore.decrement();

    if (close_semaphore.try_decrement()) {
      break;
    }

    const char *shaders = reinterpret_cast<const char *>(shared_mem.get_data());

    const char *vert_src = shaders;
    const char *frag_src = shaders + strlen(shaders) + 1;

    DefaultHash<StringRefNull> hasher;
    uint64_t vert_hash = hasher(vert_src);
    uint64_t frag_hash = hasher(frag_src);
    std::string hash_str = std::to_string(vert_hash) + "_" + std::to_string(frag_hash);
    std::string cache_path = cache_dir + SEP_STR + hash_str;

    /* TODO: This should lock the files? */
    if (BLI_exists(cache_path.c_str())) {
      /* Read cached binary. */
      fstream file(cache_path, std::ios::binary | std::ios::in | std::ios::ate);
      std::streamsize size = file.tellg();
      if (size <= compilation_subprocess_shared_memory_size) {
        file.seekg(0, std::ios::beg);
        file.read(reinterpret_cast<char *>(shared_mem.get_data()), size);
        /* Ensure it's valid. */
        if (validate_binary(shared_mem.get_data())) {
          end_semaphore.increment();
          continue;
        }
      }
      else {
        /* This should never happen, since shaders larger than the pool size should be discarded
         * and compiled in the main Blender process. */
        BLI_assert_unreachable();
      }
    }

    SubprocessShader shader(vert_src, frag_src);
    ShaderBinaryHeader *binary = shader.get_binary(shared_mem.get_data());

    end_semaphore.increment();

    fstream file(cache_path, std::ios::binary | std::ios::out);
    file.write(reinterpret_cast<char *>(shared_mem.get_data()),
               binary->size + offsetof(ShaderBinaryHeader, data_start));
  }

  GPU_exit();
  GPU_context_discard(gpu_context);
  GHOST_DisposeGPUContext(ghost_system, ghost_context);
  GHOST_DisposeSystem(ghost_system);
}

#else

#  include "BLI_assert.h"

void GPU_compilation_subprocess_run(const char *subprocess_name)
{
  BLI_assert_unreachable();
}

#endif
