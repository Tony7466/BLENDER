/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"

#include "mesh.h"

namespace blender::render::hydra {

class InstancerData : public MeshData {
 public:
  InstancerData(BlenderSceneDelegate *scene_delegate, Object *object);

  static bool is_supported(Object *object);
  static std::unique_ptr<InstancerData> create(BlenderSceneDelegate *scene_delegate,
                                               Object *object);
  static pxr::SdfPath prim_id(BlenderSceneDelegate *scene_delegate, Object *object);

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
  bool set_instances();

  Object *parent_obj_;
  pxr::VtMatrix4dArray transforms_;
};

}  // namespace blender::render::hydra
