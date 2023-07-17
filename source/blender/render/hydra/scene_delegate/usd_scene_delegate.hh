/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2023 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/usdImaging/usdImaging/delegate.h>

#include <string>

#include "settings.h"

struct Depsgraph;

namespace blender::render::hydra {

/* Populate Hydra render index using USD file export, for testing. */
class USDSceneDelegate {
  pxr::HdRenderIndex *render_index_;
  pxr::SdfPath const delegate_id_;
  pxr::UsdStageRefPtr stage_;
  std::unique_ptr<pxr::UsdImagingDelegate> delegate_;
  const SceneDelegateSettings &settings_;

  std::string temp_dir_;
  std::string temp_file_;

 public:
  USDSceneDelegate(pxr::HdRenderIndex *render_index,
                   pxr::SdfPath const &delegate_id,
                   const SceneDelegateSettings &settings_);
  ~USDSceneDelegate();

  void populate(Depsgraph *depsgraph);
};

}  // namespace blender::render::hydra
