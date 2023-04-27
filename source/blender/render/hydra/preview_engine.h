/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 public:
  using FinalEngine::FinalEngine;

  static PreviewEngine *get_instance(RenderEngine *bl_engine,
                                     const std::string &render_delegate_id);
  static void schedule_free();

  void sync(Depsgraph *depsgraph,
            bContext *context,
            pxr::HdRenderSettingsMap &render_settings) override;
  void render(Depsgraph *depsgraph) override;

 private:
  /* Singleton class instance */
  static double free_instance(uintptr_t uuid, void *user_data);
  static std::unique_ptr<PreviewEngine> instance_;

  void update(RenderEngine *bl_engine, const std::string &render_delegate_id);
  void update_render_result(std::vector<float> &pixels);
};

}  // namespace blender::render::hydra
