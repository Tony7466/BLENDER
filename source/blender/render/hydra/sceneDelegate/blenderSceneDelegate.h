/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_context.h"
#include "DEG_depsgraph.h"

#include "object.h"
#include "mesh.h"
#include "light.h"
#include "world.h"
#include "instance.h"

namespace blender::render::hydra {

class BlenderSceneDelegate : public pxr::HdSceneDelegate {
public:
  BlenderSceneDelegate(pxr::HdRenderIndex *renderIndex, pxr::SdfPath const &delegateId);
  ~BlenderSceneDelegate() override = default;

  void populate(Depsgraph *depsgraph, bContext *context);

  // delegate methods
  pxr::HdMeshTopology GetMeshTopology(pxr::SdfPath const &id) override;
  pxr::GfMatrix4d GetTransform(pxr::SdfPath const &id) override;
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::VtValue GetLightParamValue(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::HdPrimvarDescriptorVector GetPrimvarDescriptors(pxr::SdfPath const &id, pxr::HdInterpolation interpolation) override;
  pxr::SdfPath GetMaterialId(pxr::SdfPath const &rprimId) override;
  pxr::VtValue GetMaterialResource(pxr::SdfPath const &materialId) override;
  bool GetVisible(pxr::SdfPath const &id) override;

private:
  ObjectData *object_data(pxr::SdfPath const &id);
  MeshData *mesh_data(pxr::SdfPath const &id);
  LightData *light_data(pxr::SdfPath const &id);
  MaterialData *material_data(pxr::SdfPath const &id);

  void add_update_object(Object *object, bool geometry, bool transform, bool shading);
  void set_material(MeshData &mesh_data);
  void update_material(Material *material);
  void update_world();
  void update_collection(bool remove, bool visibility);

private:  
  Depsgraph *depsgraph;
  bContext *context;
  View3D *view3d;

  ObjectDataMap objects;
  MaterialDataMap materials;
  std::unique_ptr<WorldData> world_data;
  InstanceDataMap instances;
};

} // namespace blender::render::hydra
