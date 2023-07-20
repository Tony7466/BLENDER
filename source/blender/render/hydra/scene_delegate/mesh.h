/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_duplilist.h"
#include "BLI_set.hh"

#include "material.h"
#include "object.h"

namespace blender::render::hydra {

class MeshData : public ObjectData {
  struct SubMesh {
    pxr::VtIntArray face_vertex_counts;
    pxr::VtIntArray face_vertex_indices;
    pxr::VtVec3fArray normals;
    pxr::VtVec2fArray uvs;
    int mat_index = 0;
    MaterialData *mat_data = nullptr;
  };

 public:
  MeshData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  pxr::VtValue get_data(pxr::SdfPath const &id, pxr::TfToken const &key) const override;
  pxr::SdfPath material_id(pxr::SdfPath const &id) const override;
  void available_materials(Set<pxr::SdfPath> &paths) const override;

  pxr::HdMeshTopology mesh_topology(pxr::SdfPath const &id) const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;
  pxr::HdCullStyle cull_style(pxr::SdfPath const &id) const;
  bool double_sided(pxr::SdfPath const &id) const;
  void update_double_sided(MaterialData *mat_data);
  pxr::SdfPathVector submesh_paths() const;

 protected:
  void write_materials() override;

 private:
  pxr::SdfPath submesh_prim_id(int index) const;
  const SubMesh &submesh(pxr::SdfPath const &id) const;
  void write_submeshes(Mesh *mesh);
  void update_prims();

  pxr::VtVec3fArray vertices_;
  std::vector<SubMesh> submeshes_;
  int submeshes_count_ = 0;
};

}  // namespace blender::render::hydra
