/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_context.h"
#include "BLI_map.hh"
#include "DEG_depsgraph.h"

#include "CLG_log.h"

#include "curves.h"
#include "instancer.h"
#include "light.h"
#include "mesh.h"
#include "object.h"
#include "volume.h"
#include "world.h"

namespace blender::render::hydra {

extern struct CLG_LogRef *LOG_RENDER_HYDRA_SCENE;

class Engine;

class BlenderSceneDelegate : public pxr::HdSceneDelegate {
  friend ObjectData;   /* has access to instances */
  friend CurvesData;   /* has access to materials */
  friend VolumeData;   /* has access to materials */
  friend MeshData;     /* has access to materials */
  friend MaterialData; /* has access to objects and instancers */

 public:
  struct Settings {
    pxr::TfToken mx_filename_key;
    Map<pxr::TfToken, pxr::VtValue> render_tokens;
  };

  struct ShadingSettings {
    bool use_scene_lights = true;
    bool use_scene_world = true;
    std::string studiolight_name;
    float studiolight_rotation;
    float studiolight_intensity;

    bool operator==(const ShadingSettings &other);
  };

  BlenderSceneDelegate(pxr::HdRenderIndex *parent_index,
                       pxr::SdfPath const &delegate_id,
                       Engine *engine);
  ~BlenderSceneDelegate() override = default;

  /* Delegate methods */
  pxr::HdMeshTopology GetMeshTopology(pxr::SdfPath const &id) override;
  pxr::HdBasisCurvesTopology GetBasisCurvesTopology(pxr::SdfPath const &id) override;
  pxr::GfMatrix4d GetTransform(pxr::SdfPath const &id) override;
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::VtValue GetLightParamValue(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::HdPrimvarDescriptorVector GetPrimvarDescriptors(
      pxr::SdfPath const &id, pxr::HdInterpolation interpolation) override;
  pxr::SdfPath GetMaterialId(pxr::SdfPath const &rprim_id) override;
  pxr::VtValue GetMaterialResource(pxr::SdfPath const &material_id) override;
  bool GetVisible(pxr::SdfPath const &id) override;
  bool GetDoubleSided(pxr::SdfPath const &id) override;
  pxr::HdCullStyle GetCullStyle(pxr::SdfPath const &id) override;
  pxr::SdfPath GetInstancerId(pxr::SdfPath const &prim_id) override;
  pxr::SdfPathVector GetInstancerPrototypes(pxr::SdfPath const &instancer_id) override;
  pxr::VtIntArray GetInstanceIndices(pxr::SdfPath const &instancer_id,
                                     pxr::SdfPath const &prototype_id) override;
  pxr::GfMatrix4d GetInstancerTransform(pxr::SdfPath const &instancer_id) override;
  pxr::HdVolumeFieldDescriptorVector GetVolumeFieldDescriptors(
      pxr::SdfPath const &volume_id) override;

  void populate(Depsgraph *depsgraph, bContext *context);
  void clear();
  void set_setting(const std::string &key, const pxr::VtValue &val);

  Depsgraph *depsgraph = nullptr;
  bContext *context = nullptr;
  View3D *view3d = nullptr;
  Scene *scene = nullptr;
  Engine *engine;
  Settings settings;
  ShadingSettings shading_settings;

 private:
  pxr::SdfPath prim_id(ID *id, const char *prefix) const;
  pxr::SdfPath object_prim_id(Object *object) const;
  pxr::SdfPath material_prim_id(Material *mat) const;
  pxr::SdfPath instancer_prim_id(Object *object) const;
  pxr::SdfPath world_prim_id() const;

  ObjectData *object_data(pxr::SdfPath const &id) const;
  MeshData *mesh_data(pxr::SdfPath const &id) const;
  CurvesData *curves_data(pxr::SdfPath const &id) const;
  LightData *light_data(pxr::SdfPath const &id) const;
  MaterialData *material_data(pxr::SdfPath const &id) const;
  InstancerData *instancer_data(pxr::SdfPath const &id, bool child_id = false) const;
  VolumeData *volume_data(pxr::SdfPath const &id) const;

  void update_objects(Object *object);
  void update_instancers(Object *object);
  void update_world();
  void check_updates();
  void add_new_objects();
  void remove_unused_objects();
  void update_visibility();
  bool set_light_shading_settings();
  bool set_world_shading_settings();

  ObjectDataMap objects_;
  MaterialDataMap materials_;
  InstancerDataMap instancers_;
  std::unique_ptr<WorldData> world_data_;
};

}  // namespace blender::render::hydra
