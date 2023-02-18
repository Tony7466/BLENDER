/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <map>

#include <pxr/pxr.h>
#include <pxr/imaging/hd/camera.h>
#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/usd/usd/stage.h>

#include "MEM_guardedalloc.h"
#include "RNA_blender_cpp.h"

#include "object.h"
#include "mesh.h"
#include "light.h"
#include "world.h"

using namespace pxr;

namespace blender::render::hydra {

class BlenderSceneDelegate : public HdSceneDelegate {
public:
  BlenderSceneDelegate(HdRenderIndex* renderIndex, SdfPath const &delegateId);
  ~BlenderSceneDelegate() override = default;

  void Populate(BL::Depsgraph &b_deps, BL::Context &b_context);

  // delegate methods
  HdMeshTopology GetMeshTopology(SdfPath const& id) override;
  GfMatrix4d GetTransform(SdfPath const& id) override;
  VtValue Get(SdfPath const& id, TfToken const& key) override;
  VtValue GetLightParamValue(SdfPath const& id, TfToken const& key) override;
  HdPrimvarDescriptorVector GetPrimvarDescriptors(SdfPath const& id, HdInterpolation interpolation) override;
  SdfPath GetMaterialId(SdfPath const &rprimId) override;
  VtValue GetMaterialResource(SdfPath const &materialId) override;
  bool GetVisible(SdfPath const &id) override;

private:
  ObjectData *object_data(SdfPath const &id);
  MeshData *mesh_data(SdfPath const &id);
  LightData *light_data(SdfPath const &id);
  MaterialData *material_data(SdfPath const &id);
  bool supported_object(Object *object);

  void add_update_object(Object *object, bool geometry, bool transform, bool shading);
  void set_material(MeshData &mesh_data);
  void update_material(Material *material);
  void update_world();
  void update_collection();
  void update_visibility();

private:  
  BL::Depsgraph *b_depsgraph;
  BL::Context *b_context;
  View3D *view3d;
  bool is_populated;
  ObjectDataMap objects;
  MaterialDataMap materials;
  std::unique_ptr<WorldData> world_data;
};

} // namespace blender::render::hydra
