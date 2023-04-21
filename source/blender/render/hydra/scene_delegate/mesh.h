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
  MeshData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  bool update_visibility() override;

  pxr::HdMeshTopology mesh_topology() const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;
  pxr::SdfPath material_id() const;

 protected:
  void write_mesh(Mesh *mesh);
  void write_material();
  void write_uv_maps(Mesh *mesh);
  void write_normals(Mesh *mesh);

  pxr::VtIntArray face_vertex_counts_;
  pxr::VtIntArray face_vertex_indices_;
  pxr::VtVec3fArray vertices_;
  pxr::VtVec2fArray uvs_;
  pxr::VtVec3fArray normals_;
  pxr::VtMatrix4dArray instances_;

  MaterialData *mat_data_ = nullptr;
};

}  // namespace blender::render::hydra
