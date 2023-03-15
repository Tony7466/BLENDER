/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <epoxy/gl.h>

#include <pxr/imaging/hd/renderBuffer.h>

#include "engine.h"

namespace blender::render::hydra {

class GLTexture {
 public:
  GLTexture();
  ~GLTexture();
  void set_buffer(pxr::HdRenderBuffer *buffer);
  void draw(GLfloat x, GLfloat y);

 private:
  void create(pxr::HdRenderBuffer *buffer);
  void free();

  GLuint texture_id;
  int width, height, channels;
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
  std::chrono::time_point<std::chrono::steady_clock> time_begin;

  GLTexture texture;
};

}  // namespace blender::render::hydra
