/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;

  void render(Depsgraph *depsgraph) override;

 private:
  void update_render_result(std::vector<float> &pixels);
};

class PreviewEngineGPU : public FinalEngineGPU {
 public:
  using FinalEngineGPU::FinalEngineGPU;

  void render(Depsgraph *depsgraph) override;
};

}  // namespace blender::render::hydra
