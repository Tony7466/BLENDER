/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 public:
  static PreviewEngine *create(RenderEngine *bl_engine, const std::string &render_delegate_name);
  static void free();

  void render(Depsgraph *depsgraph) override;

 private:
  using FinalEngine::FinalEngine;
  static double free_instance(uintptr_t uuid, void *user_data);

  void update_render_result(std::vector<float> &pixels);

  /* Singleton class instance */
  static std::unique_ptr<PreviewEngine> instance_;
};

}  // namespace blender::render::hydra
