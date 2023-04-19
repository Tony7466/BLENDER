/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"

#include "mesh.h"

namespace blender::render::hydra {

class InstancerData : public ObjectData {
  friend BlenderSceneDelegate;

  struct Instance {
    std::unique_ptr<ObjectData> obj_data;
    pxr::VtIntArray indices;
  };

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

  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation);
  pxr::VtIntArray indices(pxr::SdfPath const &id);
  ObjectData *object_data(pxr::SdfPath const &id);
  pxr::SdfPathVector prototypes();
  void check_update(Object *object);
  void check_remove(std::set<std::string> &available_objects);
  void available_materials(std::set<pxr::SdfPath> &paths);

 private:
  void set_instances();

  pxr::TfHashMap<pxr::SdfPath, Instance, pxr::SdfPath::Hash> instances_;
  pxr::VtMatrix4dArray transforms_;
};

using InstancerDataMap =
    pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<InstancerData>, pxr::SdfPath::Hash>;

class InstanceMeshData : public MeshData {
 public:
  InstanceMeshData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &p_id);
  pxr::GfMatrix4d transform() override;
};

}  // namespace blender::render::hydra
