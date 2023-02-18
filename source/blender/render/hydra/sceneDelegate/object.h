/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/gf/matrix4d.h>
#include "pxr/base/tf/hashmap.h"

#include "DNA_object_types.h"
#include "BKE_layer.h"

#include "id.h"
#include "material.h"

namespace blender::render::hydra {

class ObjectData;
using ObjectDataMap = pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<ObjectData>, pxr::SdfPath::Hash>;

class ObjectData: public IdData {
public:
  static std::unique_ptr<ObjectData> init(pxr::HdSceneDelegate *scene_delegate, Object *object);
  static pxr::SdfPath prim_id(pxr::HdSceneDelegate *scene_delegate, Object *object);

  ObjectData(pxr::HdSceneDelegate *scene_delegate, Object *object);

  int type();
  pxr::GfMatrix4d transform();
  bool update_visibility(View3D *view3d);

  bool visible;
};



} // namespace blender::render::hydra
