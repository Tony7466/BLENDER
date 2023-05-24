/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"

#include "light.h"
#include "mesh.h"

namespace blender::render::hydra {

class InstancerData : public ObjectData {
  struct MeshInstance {
    std::unique_ptr<MeshData> data;
    pxr::VtIntArray indices;
  };

  struct LightInstance {
    std::unique_ptr<LightData> data;
    pxr::VtMatrix4dArray transforms;
    int count = 0;
  };

 public:
  InstancerData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);
  static bool is_supported(Object *object);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  bool update_visibility() override;

  pxr::GfMatrix4d get_transform(pxr::SdfPath const &id) const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;
  pxr::VtIntArray indices(pxr::SdfPath const &id) const;
  ObjectData *object_data(pxr::SdfPath const &id) const;
  pxr::SdfPathVector prototypes() const;
  void check_update(Object *object);
  void check_remove(std::set<std::string> &available_objects);
  void available_materials(std::set<pxr::SdfPath> &paths) const;
  void update_as_parent();
  void update_double_sided(MaterialData *mat_data);

 private:
  pxr::SdfPath object_prim_id(Object *object) const;
  pxr::SdfPath light_prim_id(LightInstance const &inst, int index) const;
  int light_prim_id_index(pxr::SdfPath const &id) const;
  void write_instances();
  void update_light_instance(LightInstance &inst);
  MeshInstance *mesh_instance(pxr::SdfPath const &id) const;
  LightInstance *light_instance(pxr::SdfPath const &id) const;

  pxr::TfHashMap<pxr::SdfPath, MeshInstance, pxr::SdfPath::Hash> mesh_instances_;
  pxr::TfHashMap<pxr::SdfPath, LightInstance, pxr::SdfPath::Hash> light_instances_;
  pxr::VtMatrix4dArray mesh_transforms_;
};

using InstancerDataMap =
    pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<InstancerData>, pxr::SdfPath::Hash>;

}  // namespace blender::render::hydra
