/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;

 protected:
  void notify_status(float progress, const std::string &title, const std::string &info) override;
};

}  // namespace blender::render::hydra
