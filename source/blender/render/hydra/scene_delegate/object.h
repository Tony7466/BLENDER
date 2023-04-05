/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "pxr/base/tf/hashmap.h"
#include <pxr/base/gf/matrix4d.h>

#include "BKE_layer.h"
#include "DNA_object_types.h"

#include "id.h"
#include "material.h"

namespace blender::render::hydra {

class ObjectData : public IdData {
 public:
  static bool supported(Object *object);
  static std::unique_ptr<ObjectData> create(BlenderSceneDelegate *scene_delegate, Object *object);
  static pxr::SdfPath prim_id(BlenderSceneDelegate *scene_delegate, Object *object);

  ObjectData(BlenderSceneDelegate *scene_delegate, Object *object);

  virtual pxr::GfMatrix4d transform();
  virtual bool update_visibility(View3D *view3d);

  bool visible;
};

using ObjectDataMap =
    pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<ObjectData>, pxr::SdfPath::Hash>;

}  // namespace blender::render::hydra
