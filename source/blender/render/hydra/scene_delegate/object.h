/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "pxr/base/tf/hashmap.h"
#include <pxr/base/gf/matrix4d.h>

#include "BKE_layer.h"
#include "BLI_map.hh"
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
  static bool is_visible(BlenderSceneDelegate *scene_delegate, Object *object);

  virtual bool update_visibility();
  void update_parent();

  pxr::GfMatrix4d transform;
  bool visible = true;

 protected:
  void write_transform();

  Object *parent_ = nullptr;
};

using ObjectDataMap = Map<pxr::SdfPath, std::unique_ptr<ObjectData>>;

pxr::GfMatrix4d gf_matrix_from_transform(float m[4][4]);

}  // namespace blender::render::hydra
