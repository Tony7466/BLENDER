/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "pxr/base/tf/hashmap.h"
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include "BKE_light.h"

#include "object.h"

namespace blender::render::hydra {

class InstancerData;

class LightData : public ObjectData {
  friend InstancerData;

 public:
  LightData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;

 protected:
  pxr::TfToken prim_type(Light *light);

  std::map<pxr::TfToken, pxr::VtValue> data_;
  pxr::TfToken prim_type_;
};

}  // namespace blender::render::hydra
