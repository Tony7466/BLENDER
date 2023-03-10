/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include "engine.h"

namespace blender::render::hydra {

class FinalEngine : public Engine {
public:
  using Engine::Engine;
  virtual void sync(Depsgraph *depsgraph, bContext *context, pxr::HdRenderSettingsMap &renderSettings) override;
  virtual void render(Depsgraph *b_depsgraph) override;

protected:
  pxr::GfVec2i get_resolution(Scene *scene);
  void updateRenderResult(std::map<std::string, std::vector<float>> &render_images, const std::string &layerName, int width, int height);
  void notify_status(float progress, const std::string &title, const std::string &info);

protected:
  HdRenderSettingsMap renderSettings;
};

class FinalEngineGL : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;
  void render(Depsgraph *depsgraph) override;
};

}   // namespace blender::render::hydra
