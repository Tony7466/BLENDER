/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_duplilist.h"

#include "material.h"
#include "object.h"

namespace blender::render::hydra {

class MeshData : public ObjectData {
 public:
  MeshData(BlenderSceneDelegate *scene_delegate, Object *object);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;
  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  bool update_visibility(View3D *view3d) override;

  pxr::HdMeshTopology mesh_topology();
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation);
  pxr::SdfPath material_id();

 protected:
  void set_mesh(Mesh *mesh);
  void set_material();

  pxr::VtIntArray face_vertex_counts;
  pxr::VtIntArray face_vertex_indices;
  pxr::VtVec3fArray vertices;
  pxr::VtVec3fArray normals;
  pxr::VtVec2fArray uvs;

  pxr::VtMatrix4dArray instances;

  MaterialData *mat_data;
};

}  // namespace blender::render::hydra
