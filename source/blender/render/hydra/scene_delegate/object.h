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
  ObjectData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  static std::unique_ptr<ObjectData> create(BlenderSceneDelegate *scene_delegate,
                                            Object *object,
                                            pxr::SdfPath const &prim_id);
  static bool is_supported(Object *object);

  virtual bool update_visibility();

  pxr::GfMatrix4d transform;
  bool visible;

 protected:
  void write_transform();
};

using ObjectDataMap =
    pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<ObjectData>, pxr::SdfPath::Hash>;

}  // namespace blender::render::hydra
