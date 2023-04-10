/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "final_engine.h"

namespace blender::render::hydra {

class PreviewEngine : public FinalEngine {
 private:
  /* Singleton class instance */
  static std::unique_ptr<PreviewEngine> instance;
  static double free_instance(uintptr_t uuid, void *user_data);

 public:
  static PreviewEngine *get_instance(RenderEngine *bl_engine, const std::string &render_delegate_id);
  static void schedule_free();

  using FinalEngine::FinalEngine;
  void sync(Depsgraph *depsgraph,
            bContext *context,
            pxr::HdRenderSettingsMap &render_settings) override;
  void render(Depsgraph *depsgraph) override;

 private:
  void update(RenderEngine *bl_engine, const std::string &render_delegate_id);
  void update_render_result(const std::string &layer_name,
                            int width,
                            int height,
                            std::vector<float> &pixels);

};

}  // namespace blender::render::hydra
