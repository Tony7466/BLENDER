/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include "engine.h"

namespace blender::render::hydra {

class FinalEngine : public Engine {
public:
  using Engine::Engine;
  virtual void sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, pxr::HdRenderSettingsMap &renderSettings) override;
  virtual void render(BL::Depsgraph &b_depsgraph) override;

protected:
  pxr::GfVec2i get_resolution(BL::RenderSettings b_render);
  void updateRenderResult(std::map<std::string, std::vector<float>> &render_images, const std::string &layerName, int width, int height);
  void notifyStatus(float progress, const std::string &title, const std::string &info);

protected:
  HdRenderSettingsMap renderSettings;
};

class FinalEngineGL : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;
  void render(BL::Depsgraph &b_depsgraph) override;
};

}   // namespace blender::render::hydra
