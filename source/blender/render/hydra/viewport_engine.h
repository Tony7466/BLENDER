/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <pxr/imaging/hd/renderBuffer.h>

#include "GPU_batch.h"
#include "GPU_shader.h"
#include "GPU_texture.h"

#include "engine.h"

namespace blender::render::hydra {

class DrawTexture {
 public:
  DrawTexture();
  ~DrawTexture();

  void set_buffer(pxr::HdRenderBuffer *buffer);
  void draw(GPUShader *shader, float x, float y);

 private:
  void create(pxr::HdRenderBuffer *buffer);
  void free();

  GPUTexture *texture_;
  GPUBatch *batch_;
  int width_, height_, channels_;
};

class ViewportEngine : public Engine {
 public:
  using Engine::Engine;

  void sync(Depsgraph *depsgraph,
            bContext *context,
            pxr::HdRenderSettingsMap &render_settings) override;
  void render(Depsgraph *depsgraph) override;
  void render(Depsgraph *depsgraph, bContext *context);

 private:
  void notify_status(const std::string &title, const std::string &info);

 private:
  std::chrono::time_point<std::chrono::steady_clock> time_begin_;

  DrawTexture draw_texture_;
};

}  // namespace blender::render::hydra
