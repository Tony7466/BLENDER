/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "pxr/base/tf/hashmap.h"
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include "BKE_light.h"

#include "object.h"

namespace blender::render::hydra {

class LightData : public ObjectData {
 public:
  LightData(BlenderSceneDelegate *scene_delegate, Object *object);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;
  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  bool update_visibility(View3D *view3d) override;

 private:
  std::map<pxr::TfToken, pxr::VtValue> data;
  pxr::TfToken p_type;
  pxr::TfToken prim_type(Light *light);
};

}  // namespace blender::render::hydra
