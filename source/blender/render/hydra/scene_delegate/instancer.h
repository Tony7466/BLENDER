/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"

#include "mesh.h"

namespace blender::render::hydra {

class InstancerData : public MeshData {
 public:
  static bool supported(Object *object);
  static std::unique_ptr<InstancerData> create(BlenderSceneDelegate *scene_delegate,
                                               Object *object);
  static pxr::SdfPath prim_id(BlenderSceneDelegate *scene_delegate, Object *object);

  InstancerData(BlenderSceneDelegate *scene_delegate, Object *object);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;
  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  pxr::GfMatrix4d transform() override;
  bool update_visibility(View3D *view3d) override;

  pxr::HdPrimvarDescriptorVector instancer_primvar_descriptors(pxr::HdInterpolation interpolation);
  pxr::VtIntArray instance_indices();

  bool is_base(Object *object) const;
  pxr::SdfPath instancer_id;

 private:
  Object *parent_obj;
  pxr::VtMatrix4dArray transforms;

  bool set_instances();
};

}  // namespace blender::render::hydra
