/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "object.h"

namespace blender::render::hydra {

class MeshData: public ObjectData {
public:
  MeshData(BlenderSceneDelegate *scene_delegate, Object *object);

  pxr::VtValue get_data(pxr::TfToken const &key) override;

  void insert_prim() override;
  void remove_prim() override;
  void mark_prim_dirty(DirtyBits dirty_bits) override;

  Material *material();
  pxr::HdMeshTopology mesh_topology();
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation);

  pxr::SdfPath material_id;

 private:
  void set_mesh(Mesh *mesh);

  pxr::VtIntArray face_vertex_counts;
  pxr::VtIntArray face_vertex_indices;
  pxr::VtVec3fArray vertices;
  pxr::VtVec3fArray normals;
  pxr::VtVec2fArray uvs;
};

} // namespace blender::render::hydra
