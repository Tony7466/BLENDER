/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "CLG_log.h"
#include "GHOST_C-api.h"
#include "GPU_context.hh"
#include "GPU_init_exit.hh"
#include "opengl/ipc.hh"
#include <epoxy/gl.h>
#include <string>

class SubprocessShader {
  GLuint vert = 0;
  GLuint frag = 0;
  GLuint program = 0;
  bool success = false;

 public:
  SubprocessShader(const char *vert_src, const char *frag_src)
  {
    GLint status;
    static char log[5000] = "";

    vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vert_src, nullptr);
    glCompileShader(vert);
    glGetShaderiv(vert, GL_COMPILE_STATUS, &status);
    if (!status) {
      glGetShaderInfoLog(vert, sizeof(log), nullptr, log);
      fprintf(stderr, log);
      return;
    }

    frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &frag_src, nullptr);
    glCompileShader(frag);
    glGetShaderiv(frag, GL_COMPILE_STATUS, &status);
    if (!status) {
      glGetShaderInfoLog(frag, sizeof(log), nullptr, log);
      fprintf(stderr, log);
      return;
    }

    program = glCreateProgram();
    glAttachShader(program, vert);
    glAttachShader(program, frag);
    glLinkProgram(program);
    glGetProgramiv(program, GL_COMPILE_STATUS, &status);
    if (!status) {
      glGetProgramInfoLog(program, sizeof(log), nullptr, log);
      fprintf(stderr, log);
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

  void load_binary(void *memory)
  {
    struct ShaderBinary {
      GLint size;
      GLuint format;
      GLubyte data_start;
    };
    ShaderBinary *bin = reinterpret_cast<ShaderBinary *>(memory);

    if (success) {
      glGetProgramiv(program, GL_PROGRAM_BINARY_LENGTH, &bin->size);
      glGetProgramBinary(program, bin->size, nullptr, &bin->format, &bin->data_start);
    }
    else {
      bin->size = 0;
      bin->format = 0;
    }
  }
};

void GPU_compilation_subprocess_run(const char *subprocess_name)
{
  CLG_init();

  ipc_sharedmemory_ shared_mem = {0};
  ipc_mem_init(&shared_mem, subprocess_name, 1024 * 1024 * 2 /*2mb TODO: pass as parameter. */);
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

  while (true) {
    ipc_sem_decrement(&start_semaphore);

    if (ipc_sem_try_decrement(&close_semaphore)) {
      break;
    }

    const char *shaders = reinterpret_cast<const char *>(shared_mem.data);

    const char *vert_src = shaders;
    const char *frag_src = shaders + strlen(shaders) + 1;

    SubprocessShader shader(vert_src, frag_src);
    shader.load_binary(shared_mem.data);

    ipc_sem_increment(&end_semaphore);
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
