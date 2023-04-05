/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_object.h"

#include "../utils.h"
#include "blender_scene_delegate.h"
#include "mesh.h"

namespace blender::render::hydra {

MeshData::MeshData(BlenderSceneDelegate *scene_delegate, Object *object)
    : ObjectData(scene_delegate, object), mat_data(nullptr)
{
  CLOG_INFO(LOG_BSD, 2, "%s, id=%s", id->name, p_id.GetText());
}

pxr::VtValue MeshData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  if (key == pxr::HdTokens->points) {
    ret = vertices;
  }
  else if (key == pxr::HdTokens->normals) {
    ret = normals;
  }
  else if (key == pxr::HdPrimvarRoleTokens->textureCoordinate) {
    ret = uvs;
  }
  else if (key == pxr::HdInstancerTokens->instanceTransform) {
    ret = instances;
  }
  return ret;
}

bool MeshData::update_visibility(View3D *view3d)
{
  bool ret = ObjectData::update_visibility(view3d);
  if (ret) {
    scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
        p_id, pxr::HdChangeTracker::DirtyVisibility);
  }
  return ret;
}

pxr::HdMeshTopology MeshData::mesh_topology()
{
  return pxr::HdMeshTopology(pxr::PxOsdOpenSubdivTokens->none,
                             pxr::HdTokens->rightHanded,
                             face_vertex_counts,
                             face_vertex_indices);
}

pxr::HdPrimvarDescriptorVector MeshData::primvar_descriptors(pxr::HdInterpolation interpolation)
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    if (!vertices.empty()) {
      primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
    }
  }
  else if (interpolation == pxr::HdInterpolationFaceVarying) {
    if (!normals.empty()) {
      primvars.emplace_back(
          pxr::HdTokens->normals, interpolation, pxr::HdPrimvarRoleTokens->normal);
    }
    if (!uvs.empty()) {
      primvars.emplace_back(pxr::HdPrimvarRoleTokens->textureCoordinate,
                            interpolation,
                            pxr::HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

pxr::SdfPath MeshData::material_id()
{
  if (!mat_data) {
    return pxr::SdfPath();
  }
  return mat_data->p_id;
}

void MeshData::set_mesh(Mesh *mesh)
{
  face_vertex_counts.clear();
  face_vertex_indices.clear();
  vertices.clear();
  normals.clear();
  uvs.clear();

  BKE_mesh_calc_normals_split(mesh);
  int tris_len = BKE_mesh_runtime_looptri_len(mesh);
  if (tris_len == 0) {
    return;
  }

  blender::Span<MLoopTri> loopTris = mesh->looptris();

  /* face_vertex_counts */
  face_vertex_counts = pxr::VtIntArray(tris_len, 3);

  /* face_vertex_indices */
  blender::Span<int> corner_verts = mesh->corner_verts();
  face_vertex_indices.reserve(loopTris.size() * 3);
  for (MLoopTri lt : loopTris) {
    face_vertex_indices.push_back(corner_verts[lt.tri[0]]);
    face_vertex_indices.push_back(corner_verts[lt.tri[1]]);
    face_vertex_indices.push_back(corner_verts[lt.tri[2]]);
  }

  /* vertices */
  vertices.reserve(mesh->totvert);
  blender::Span<blender::float3> verts = mesh->vert_positions();
  for (blender::float3 v : verts) {
    vertices.push_back(pxr::GfVec3f(v.x, v.y, v.z));
  }

  /* normals */
  const float(*lnors)[3] = (float(*)[3])CustomData_get_layer(&mesh->ldata, CD_NORMAL);
  if (lnors) {
    normals.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      normals.push_back(pxr::GfVec3f(lnors[lt.tri[0]]));
      normals.push_back(pxr::GfVec3f(lnors[lt.tri[1]]));
      normals.push_back(pxr::GfVec3f(lnors[lt.tri[2]]));
    }
  }

  /* uvs*/
  const float(*luvs)[2] = (float(*)[2])CustomData_get_layer(&mesh->ldata, CD_PROP_FLOAT2);
  if (luvs) {
    uvs.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      uvs.push_back(pxr::GfVec2f(luvs[lt.tri[0]]));
      uvs.push_back(pxr::GfVec2f(luvs[lt.tri[1]]));
      uvs.push_back(pxr::GfVec2f(luvs[lt.tri[2]]));
    }
  }
}

void MeshData::set_material()
{
  Object *object = (Object *)id;
  Material *mat = nullptr;
  if (BKE_object_material_count_eval(object) > 0) {
    mat = BKE_object_material_get_eval(object, object->actcol);
  }

  if (!mat) {
    mat_data = nullptr;
    return;
  }
  pxr::SdfPath id = MaterialData::prim_id(scene_delegate, mat);
  mat_data = scene_delegate->material_data(id);
  if (!mat_data) {
    scene_delegate->materials[id] = MaterialData::create(scene_delegate, mat);
    mat_data = scene_delegate->material_data(id);
  }
}

void MeshData::init()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id->name);

  Object *object = (Object *)id;
  if (object->type == OB_MESH && object->mode == OB_MODE_OBJECT &&
      BLI_listbase_is_empty(&object->modifiers)) {
    set_mesh((Mesh *)object->data);
  }
  else {
    Mesh *mesh = BKE_object_to_mesh(nullptr, object, false);
    if (mesh) {
      set_mesh(mesh);
    }
    BKE_object_to_mesh_clear(object);
  }

  set_material();
}

void MeshData::insert()
{
  if (face_vertex_counts.empty()) {
    return;
  }

  CLOG_INFO(LOG_BSD, 2, "%s", id->name);
  scene_delegate->GetRenderIndex().InsertRprim(pxr::HdPrimTypeTokens->mesh, scene_delegate, p_id);
}

void MeshData::remove()
{
  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    return;
  }

  CLOG_INFO(LOG_BSD, 2, "%s", id->name);
  scene_delegate->GetRenderIndex().RemoveRprim(p_id);
}

void MeshData::update()
{
  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  Object *object = (Object *)id;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    bits = pxr::HdChangeTracker::AllDirty;
  }
  else {
    if (id->recalc & ID_RECALC_SHADING) {
      set_material();
      bits |= pxr::HdChangeTracker::DirtyMaterialId;
    }
    if (id->recalc & ID_RECALC_TRANSFORM) {
      bits |= pxr::HdChangeTracker::DirtyTransform;
    }
  }

  if (bits == pxr::HdChangeTracker::Clean) {
    return;
  }

  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    insert();
    return;
  }

  if (face_vertex_counts.empty()) {
    /* Remove prim without faces */
    remove();
    return;
  }
  CLOG_INFO(LOG_BSD, 2, "%s", id->name);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(p_id, bits);
}

}  // namespace blender::render::hydra
