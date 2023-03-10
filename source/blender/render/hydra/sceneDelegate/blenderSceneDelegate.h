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

namespace blender::render::hydra {

class BlenderSceneDelegate : public pxr::HdSceneDelegate {
public:
  enum class EngineType {
    Viewport = 1,
    Final,
    Preview
  };

  BlenderSceneDelegate(pxr::HdRenderIndex *render_index, pxr::SdfPath const &delegateId, BlenderSceneDelegate::EngineType engine_type);
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
  pxr::SdfPath GetInstancerId(pxr::SdfPath const &primId) override;
  pxr::SdfPathVector GetInstancerPrototypes(pxr::SdfPath const &instancerId) override;
  pxr::VtIntArray GetInstanceIndices(pxr::SdfPath const &instancerId, pxr::SdfPath const &prototypeId) override;
  pxr::GfMatrix4d GetInstancerTransform(pxr::SdfPath const &instancerId);
  size_t SampleInstancerTransform(pxr::SdfPath const &instancerId, size_t maxSampleCount,
                                  float *sampleTimes, pxr::GfMatrix4d *sampleValues) override;
  size_t SamplePrimvar(pxr::SdfPath const &id, pxr::TfToken const &key, size_t maxSampleCount,
                       float *sampleTimes, pxr::VtValue *sampleValues) override;

  EngineType engine_type;

private:
  ObjectData *object_data(pxr::SdfPath const &id);
  MeshData *mesh_data(pxr::SdfPath const &id);
  LightData *light_data(pxr::SdfPath const &id);
  MaterialData *material_data(pxr::SdfPath const &id);

  void add_update_object(Object *object, bool geometry, bool transform, bool shading);
  void add_update_instance(DupliObject *dupli);
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
};

} // namespace blender::render::hydra
