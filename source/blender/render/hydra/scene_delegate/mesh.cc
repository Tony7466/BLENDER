/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_object.h"

#include "blender_scene_delegate.h"
#include "mesh.h"

namespace blender::render::hydra {

MeshData::MeshData(BlenderSceneDelegate *scene_delegate,
                   Object *object,
                   pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

void MeshData::init()
{
  ID_LOG(2, "");

  Object *object = (Object *)id;
  if (object->type == OB_MESH && object->mode == OB_MODE_OBJECT &&
      BLI_listbase_is_empty(&object->modifiers))
  {
    write_mesh((Mesh *)object->data);
  }
  else {
    Mesh *mesh = BKE_object_to_mesh(nullptr, object, false);
    if (mesh) {
      write_mesh(mesh);
    }
    BKE_object_to_mesh_clear(object);
  }

  write_material();
  write_transform();
}

void MeshData::insert()
{
  if (face_vertex_counts_.empty()) {
    return;
  }

  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertRprim(
      pxr::HdPrimTypeTokens->mesh, scene_delegate_, prim_id);
}

void MeshData::remove()
{
  if (!scene_delegate_->GetRenderIndex().HasRprim(prim_id)) {
    return;
  }

  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 2, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveRprim(prim_id);
}

void MeshData::update()
{
  Object *object = (Object *)id;
  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    bits = pxr::HdChangeTracker::AllDirty;
  }
  else {
    if (id->recalc & ID_RECALC_SHADING) {
      write_material();
      bits |= pxr::HdChangeTracker::DirtyMaterialId | pxr::HdChangeTracker::DirtyDoubleSided;
    }
    if (id->recalc & ID_RECALC_TRANSFORM) {
      write_transform();
      bits |= pxr::HdChangeTracker::DirtyTransform;
    }
  }

  if (bits == pxr::HdChangeTracker::Clean) {
    return;
  }

  if (!scene_delegate_->GetRenderIndex().HasRprim(prim_id)) {
    insert();
    return;
  }

  if (face_vertex_counts_.empty()) {
    /* Remove prim without faces */
    remove();
    return;
  }
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(prim_id, bits);
}

pxr::VtValue MeshData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  if (key == pxr::HdTokens->points) {
    ret = vertices_;
  }
  else if (key == pxr::HdTokens->normals) {
    ret = normals_;
  }
  else if (key == pxr::HdPrimvarRoleTokens->textureCoordinate) {
    ret = uvs_;
  }
  return ret;
}

bool MeshData::update_visibility()
{
  bool ret = ObjectData::update_visibility();
  if (ret) {
    ID_LOG(2, "");
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
        prim_id, pxr::HdChangeTracker::DirtyVisibility);
  }
  return ret;
}

pxr::HdMeshTopology MeshData::mesh_topology() const
{
  return pxr::HdMeshTopology(pxr::PxOsdOpenSubdivTokens->none,
                             pxr::HdTokens->rightHanded,
                             face_vertex_counts_,
                             face_vertex_indices_);
}

pxr::HdPrimvarDescriptorVector MeshData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    if (!vertices_.empty()) {
      primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
    }
  }
  else if (interpolation == pxr::HdInterpolationFaceVarying) {
    if (!normals_.empty()) {
      primvars.emplace_back(
          pxr::HdTokens->normals, interpolation, pxr::HdPrimvarRoleTokens->normal);
    }
    if (!uvs_.empty()) {
      primvars.emplace_back(pxr::HdPrimvarRoleTokens->textureCoordinate,
                            interpolation,
                            pxr::HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

pxr::SdfPath MeshData::material_id() const
{
  if (!mat_data_) {
    return pxr::SdfPath();
  }
  return mat_data_->prim_id;
}

bool MeshData::double_sided() const
{
  if (mat_data_) {
    return mat_data_->double_sided;
  }
  return true;
}

void MeshData::update_double_sided(MaterialData *mat_data)
{
  if (mat_data_ == mat_data) {
    ID_LOG(2, "");
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
        prim_id, pxr::HdChangeTracker::DirtyDoubleSided);
  }
}

void MeshData::write_mesh(Mesh *mesh)
{
  face_vertex_counts_.clear();
  face_vertex_indices_.clear();
  vertices_.clear();
  normals_.clear();
  uvs_.clear();

  BKE_mesh_calc_normals_split(mesh);
  int tris_len = BKE_mesh_runtime_looptri_len(mesh);
  if (tris_len == 0) {
    return;
  }

  blender::Span<MLoopTri> loopTris = mesh->looptris();

  /* face_vertex_counts */
  face_vertex_counts_ = pxr::VtIntArray(tris_len, 3);

  /* face_vertex_indices */
  blender::Span<int> corner_verts = mesh->corner_verts();
  face_vertex_indices_.reserve(loopTris.size() * 3);
  for (MLoopTri lt : loopTris) {
    face_vertex_indices_.push_back(corner_verts[lt.tri[0]]);
    face_vertex_indices_.push_back(corner_verts[lt.tri[1]]);
    face_vertex_indices_.push_back(corner_verts[lt.tri[2]]);
  }

  /* vertices */
  vertices_.reserve(mesh->totvert);
  blender::Span<blender::float3> verts = mesh->vert_positions();
  for (blender::float3 v : verts) {
    vertices_.push_back(pxr::GfVec3f(v.x, v.y, v.z));
  }

  write_normals(mesh);
  write_uv_maps(mesh);
}

void MeshData::write_material()
{
  Object *object = (Object *)id;
  Material *mat = nullptr;
  if (BKE_object_material_count_eval(object) > 0) {
    mat = BKE_object_material_get_eval(object, object->actcol);
  }

  if (!mat) {
    mat_data_ = nullptr;
    return;
  }
  pxr::SdfPath p_id = scene_delegate_->material_prim_id(mat);
  mat_data_ = scene_delegate_->material_data(p_id);
  if (!mat_data_) {
    scene_delegate_->materials_[p_id] = std::make_unique<MaterialData>(scene_delegate_, mat, p_id);
    mat_data_ = scene_delegate_->material_data(p_id);
    mat_data_->init();
    mat_data_->insert();
  }
}

void MeshData::write_uv_maps(Mesh *mesh)
{
  blender::Span<MLoopTri> loopTris = mesh->looptris();
  const float(*luvs)[2] = (float(*)[2])CustomData_get_layer(&mesh->ldata, CD_PROP_FLOAT2);
  if (luvs) {
    uvs_.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      uvs_.push_back(pxr::GfVec2f(luvs[lt.tri[0]]));
      uvs_.push_back(pxr::GfVec2f(luvs[lt.tri[1]]));
      uvs_.push_back(pxr::GfVec2f(luvs[lt.tri[2]]));
    }
  }
}

void MeshData::write_normals(Mesh *mesh)
{
  blender::Span<MLoopTri> loopTris = mesh->looptris();
  const float(*lnors)[3] = (float(*)[3])CustomData_get_layer(&mesh->ldata, CD_NORMAL);
  if (lnors) {
    normals_.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      normals_.push_back(pxr::GfVec3f(lnors[lt.tri[0]]));
      normals_.push_back(pxr::GfVec3f(lnors[lt.tri[1]]));
      normals_.push_back(pxr::GfVec3f(lnors[lt.tri[2]]));
    }
  }
}

}  // namespace blender::render::hydra
