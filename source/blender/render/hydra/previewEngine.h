/* SPDX-License-Identifier: Apache-2.0
* Copyright 2011-2022 Blender Foundation */

#pragma once

#include "finalEngine.h"

namespace blender::render::hydra {

  class PreviewEngine : public FinalEngine {
  public:
    using FinalEngine::FinalEngine;
    void sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, pxr::HdRenderSettingsMap &renderSettings) override;
    void render(BL::Depsgraph &b_depsgraph) override;

  protected:
    void updateRenderResult(const std::string &layerName, int width, int height, std::vector<float> &pixels);

  protected:
    HdRenderSettingsMap renderSettings;
  };

}   // namespace blender::render::hydra
