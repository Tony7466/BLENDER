/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;
  void sync(Depsgraph *depsgraph,
            bContext *context,
            pxr::HdRenderSettingsMap &render_settings) override;
  void render(Depsgraph *depsgraph) override;

 protected:
  void update_render_result(const std::string &layer_name,
                            int width,
                            int height,
                            std::vector<float> &pixels);

 protected:
  pxr::HdRenderSettingsMap render_settings;
};

}  // namespace blender::render::hydra
