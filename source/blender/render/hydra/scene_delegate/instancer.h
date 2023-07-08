/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"
#include "BLI_map.hh"
#include "BLI_set.hh"

#include "light.h"
#include "mesh.h"

namespace blender::render::hydra {

class InstancerData : public IdData {
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
  InstancerData(BlenderSceneDelegate *scene_delegate, pxr::SdfPath const &prim_id);
  static bool is_instance_supported(Object *object);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  pxr::GfMatrix4d get_transform(pxr::SdfPath const &id) const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;
  pxr::VtIntArray indices(pxr::SdfPath const &id) const;
  ObjectData *object_data(pxr::SdfPath const &id) const;
  pxr::SdfPathVector prototypes() const;
  void available_materials(Set<pxr::SdfPath> &paths) const;
  void update_double_sided(MaterialData *mat_data);

  /* Following update functions are working together:
       pre_update()
         update_instance()
         update_instance()
         ...
       post_update() */
  void pre_update();
  void update_instance(Object *parent_ob, DupliObject *dupli);
  void post_update();

 private:
  pxr::SdfPath object_prim_id(Object *object) const;
  pxr::SdfPath light_prim_id(LightInstance const &inst, int index) const;
  int light_prim_id_index(pxr::SdfPath const &id) const;
  void update_light_instance(LightInstance &inst);
  MeshInstance *mesh_instance(pxr::SdfPath const &id) const;
  LightInstance *light_instance(pxr::SdfPath const &id) const;

  Map<pxr::SdfPath, MeshInstance> mesh_instances_;
  Map<pxr::SdfPath, LightInstance> light_instances_;
  pxr::VtMatrix4dArray mesh_transforms_;
};

}  // namespace blender::render::hydra
