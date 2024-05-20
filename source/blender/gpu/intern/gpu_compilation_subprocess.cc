/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GPU_compilation_subprocess.hh"

#ifdef WITH_OPENGL_BACKEND

#  include "BLI_hash.hh"
#  include "CLG_log.h"
#  include "GHOST_C-api.h"
#  include "GPU_context.hh"
#  include "GPU_init_exit.hh"
#  include "opengl/ipc.hh"
#  include <epoxy/gl.h>
#  include <filesystem>
#  include <fstream>
#  include <iostream>
#  include <string>

namespace fs = std::filesystem;

namespace blender::gpu {

class SubprocessShader {
  GLuint vert = 0;
  GLuint frag = 0;
  GLuint program = 0;
  bool success = false;

 public:
  SubprocessShader(const char *vert_src, const char *frag_src)
  {
    GLint status;

    vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vert_src, nullptr);
    glCompileShader(vert);
    glGetShaderiv(vert, GL_COMPILE_STATUS, &status);
    if (!status) {
      return;
    }

    frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &frag_src, nullptr);
    glCompileShader(frag);
    glGetShaderiv(frag, GL_COMPILE_STATUS, &status);
    if (!status) {
      return;
    }

    program = glCreateProgram();
    glAttachShader(program, vert);
    glAttachShader(program, frag);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (!status) {
      return;
    }

    success = true;
  }

  ~SubprocessShader()
  {
    glDeleteShader(vert);
    glDeleteShader(frag);
    glDeleteProgram(program);
  }

  ShaderBinary *load_binary(void *memory)
  {
    ShaderBinary *bin = reinterpret_cast<ShaderBinary *>(memory);

    if (success && (bin->size + sizeof(ShaderBinary) < ShaderBinary::max_data_size)) {
      glGetProgramiv(program, GL_PROGRAM_BINARY_LENGTH, &bin->size);
      glGetProgramBinary(program, bin->size, nullptr, &bin->format, &bin->data_start);
    }
    else {
      bin->size = 0;
      bin->format = 0;
    }

    return bin;
  }
};

/* Check if the binary is valid and can be loaded by the driver. */
static bool validate_binary(void *binary)
{
  ShaderBinary *bin = reinterpret_cast<ShaderBinary *>(binary);
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

  ipc_sharedmemory_ shared_mem = {0};
  ipc_mem_init(&shared_mem, subprocess_name, ShaderBinary::max_data_size);
  if (ipc_mem_open_existing(&shared_mem) != 0) {
    return;
  }

  ipc_sharedsemaphore start_semaphore = {0};
  std::string start_name = std::string(subprocess_name) + "_START";
  ipc_sem_init(&start_semaphore, start_name.c_str());
  ipc_sem_create(&start_semaphore, 0);

  ipc_sharedsemaphore end_semaphore = {0};
  std::string end_name = std::string(subprocess_name) + "_END";
  ipc_sem_init(&end_semaphore, end_name.c_str());
  ipc_sem_create(&end_semaphore, 0);

  ipc_sharedsemaphore close_semaphore = {0};
  std::string close_name = std::string(subprocess_name) + "_CLOSE";
  ipc_sem_init(&close_semaphore, close_name.c_str());
  ipc_sem_create(&close_semaphore, 0);

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

  fs::path cache_dir = fs::temp_directory_path() / "BLENDER_SHADER_CACHE";
  fs::create_directory(cache_dir);

  while (true) {
    ipc_sem_decrement(&start_semaphore);

    if (ipc_sem_try_decrement(&close_semaphore)) {
      break;
    }

    const char *shaders = reinterpret_cast<const char *>(shared_mem.data);

    const char *vert_src = shaders;
    const char *frag_src = shaders + strlen(shaders) + 1;

    DefaultHash<StringRefNull> hasher;
    uint64_t vert_hash = hasher(vert_src);
    uint64_t frag_hash = hasher(frag_src);
    std::string hash_str = std::to_string(vert_hash) + "_" + std::to_string(frag_hash);
    fs::path cache_path = cache_dir / hash_str;

    /* TODO: This should lock the files. */
    if (fs::exists(cache_path)) {
      /* Read cached binary. */
      std::ifstream file(cache_path, std::ios::binary | std::ios::in | std::ios::ate);
      std::streamsize size = file.tellg();
      file.seekg(0, std::ios::beg);
      file.read(reinterpret_cast<char *>(shared_mem.data), size);
      /* Ensure it's valid. */
      if (validate_binary(shared_mem.data)) {
        ipc_sem_increment(&end_semaphore);
        continue;
      }
    }

    SubprocessShader shader(vert_src, frag_src);
    ShaderBinary *binary = shader.load_binary(shared_mem.data);

    ipc_sem_increment(&end_semaphore);

    std::ofstream file(cache_path, std::ios::binary | std::ios::out);
    file.write(reinterpret_cast<char *>(shared_mem.data),
               binary->size + offsetof(ShaderBinary, data_start));
  }

  GPU_exit();
  GPU_context_discard(gpu_context);
  GHOST_DisposeGPUContext(ghost_system, ghost_context);
  GHOST_DisposeSystem(ghost_system);

  ipc_mem_close(&shared_mem, false);
  ipc_sem_close(&start_semaphore);
  ipc_sem_close(&end_semaphore);
  ipc_sem_close(&close_semaphore);
}

#else

#  include "BLI_assert.h"

void GPU_compilation_subprocess_run(const char *subprocess_name)
{
  BLI_assert_unreachable();
}

#endif
