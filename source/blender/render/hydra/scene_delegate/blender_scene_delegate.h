/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_context.h"
#include "DEG_depsgraph.h"

#include "CLG_log.h"

#include "light.h"
#include "mesh.h"
#include "object.h"
#include "world.h"

namespace blender::render::hydra {

extern struct CLG_LogRef *LOG_BSD;  /* BSD - Blender Scene Delegate */

class BlenderSceneDelegate : public pxr::HdSceneDelegate {
 public:
  enum class EngineType { VIEWPORT = 1, FINAL, PREVIEW };

  BlenderSceneDelegate(pxr::HdRenderIndex *render_index,
                       pxr::SdfPath const &delegate_id,
                       BlenderSceneDelegate::EngineType engine_type);
  ~BlenderSceneDelegate() override = default;

  void populate(Depsgraph *depsgraph, bContext *context);

  // delegate methods
  pxr::HdMeshTopology GetMeshTopology(pxr::SdfPath const &id) override;
  pxr::GfMatrix4d GetTransform(pxr::SdfPath const &id) override;
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::VtValue GetLightParamValue(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::HdPrimvarDescriptorVector GetPrimvarDescriptors(
      pxr::SdfPath const &id, pxr::HdInterpolation interpolation) override;
  pxr::SdfPath GetMaterialId(pxr::SdfPath const &rprim_id) override;
  pxr::VtValue GetMaterialResource(pxr::SdfPath const &material_id) override;
  bool GetVisible(pxr::SdfPath const &id) override;
  pxr::SdfPath GetInstancerId(pxr::SdfPath const &prim_id) override;
  pxr::SdfPathVector GetInstancerPrototypes(pxr::SdfPath const &instancer_id) override;
  pxr::VtIntArray GetInstanceIndices(pxr::SdfPath const &instancer_id,
                                     pxr::SdfPath const &prototype_id) override;
  pxr::GfMatrix4d get_instancer_transform(pxr::SdfPath const &instancer_id);
  size_t SampleInstancerTransform(pxr::SdfPath const &instancer_id,
                                  size_t max_sample_count,
                                  float *sample_times,
                                  pxr::GfMatrix4d *sample_values) override;
  size_t SamplePrimvar(pxr::SdfPath const &id,
                       pxr::TfToken const &key,
                       size_t max_sample_count,
                       float *sample_times,
                       pxr::VtValue *sample_values) override;

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

}  // namespace blender::render::hydra
