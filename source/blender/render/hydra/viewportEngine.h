/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <epoxy/gl.h>

#include <pxr/imaging/hd/renderBuffer.h>

#include "engine.h"

namespace blender::render::hydra {

class GLTexture
{
public:
  GLTexture();
  ~GLTexture();
  void setBuffer(pxr::HdRenderBuffer *buffer);
  void draw(GLfloat x, GLfloat y);

private:
  void create(pxr::HdRenderBuffer *buffer);
  void free();

  GLuint textureId;
  int width, height, channels;
};

class ViewportEngine : public Engine {
public:
  using Engine::Engine;
  void sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, pxr::HdRenderSettingsMap &renderSettings) override;
  void render(BL::Depsgraph &b_depsgraph) override;
  void render(BL::Depsgraph &b_depsgraph, BL::Context &b_context);

private:
  void notifyStatus(const std::string &title, const std::string &info);

private:
  std::chrono::time_point<std::chrono::steady_clock> timeBegin;

  GLTexture texture;
};

}   // namespace blender::render::hydra
