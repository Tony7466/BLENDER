/* SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation */

#pragma once

#include "engine.h"

namespace blender::render::hydra {

class FinalEngine : public Engine {
 protected:
  std::string scene_name_;
  std::string layer_name_;
  pxr::GfVec2i resolution_;

 public:
  using Engine::Engine;

  void render(Depsgraph *depsgraph) override;

 protected:
  virtual void notify_status(float progress, const std::string &title, const std::string &info);
  void update_render_result();
};

}  // namespace blender::render::hydra
