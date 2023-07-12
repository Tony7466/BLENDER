/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "engine.h"

namespace blender::render::hydra {

class FinalEngine : public Engine {
 public:
  using Engine::Engine;

  void render(Depsgraph *b_depsgraph) override;

 protected:
  void update_render_result();
  void notify_status(float progress, const std::string &title, const std::string &info);
  void prepare_for_render(Depsgraph *depsgraph);

  pxr::HdRenderSettingsMap render_settings_;
  pxr::HdTaskSharedPtrVector tasks_;
  std::string scene_name_;
  std::string layer_name_;
  std::map<std::string, std::vector<float>> render_images_;
  pxr::GfVec2i resolution_;
};

class FinalEngineGPU : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;

  void render(Depsgraph *depsgraph) override;
};

}  // namespace blender::render::hydra
