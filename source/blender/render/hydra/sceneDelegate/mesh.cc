/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "glog/logging.h"

#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_object.h"
#include "BKE_material.h"


#include "mesh.h"

using namespace pxr;

namespace blender::render::hydra {

MeshData::MeshData(pxr::HdSceneDelegate *scene_delegate, Object *object)
  : ObjectData(scene_delegate, object)
{
  if (object->type == OB_MESH && object->mode == OB_MODE_OBJECT &&
      BLI_listbase_is_empty(&object->modifiers)) {
    set_mesh((Mesh *)object->data);
  }
  else {
    Mesh *mesh = BKE_object_to_mesh(nullptr, object, false);
    set_mesh(mesh);
    BKE_object_to_mesh_clear(object);
  }
}

VtValue MeshData::get_data(TfToken const &key)
{
  VtValue ret;
  if (key == HdTokens->points) {
    ret = vertices;
  }
  else if (key == HdTokens->normals) {
    ret = normals;
  }
  else if (key == HdPrimvarRoleTokens->textureCoordinate) {
    ret = uvs;
  }
  return ret;
}

Material *MeshData::material()
{
  Object *object = (Object *)id;
  if (BKE_object_material_count_eval(object) == 0) {
    return nullptr;
  }
  return BKE_object_material_get_eval(object, object->actcol);
}

HdMeshTopology MeshData::mesh_topology()
{
  return HdMeshTopology(PxOsdOpenSubdivTokens->catmullClark, HdTokens->rightHanded,
                        face_vertex_counts, face_vertex_indices);
}

HdPrimvarDescriptorVector MeshData::primvar_descriptors(HdInterpolation interpolation)
{
  HdPrimvarDescriptorVector primvars;
  if (interpolation == HdInterpolationVertex) {
    if (!vertices.empty()) {
      primvars.emplace_back(HdTokens->points, interpolation, HdPrimvarRoleTokens->point);
    }
  }
  else if (interpolation == HdInterpolationFaceVarying) {
    if (!vertices.empty()) {
      primvars.emplace_back(HdTokens->normals, interpolation, HdPrimvarRoleTokens->normal);
    }
    if (!uvs.empty()) {
      primvars.emplace_back(HdPrimvarRoleTokens->textureCoordinate, interpolation,
                            HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

void MeshData::set_mesh(Mesh *mesh)
{
  BKE_mesh_calc_normals_split(mesh);
  int tris_len = BKE_mesh_runtime_looptri_len(mesh);
  if (tris_len == 0) {
    return;
  }

  blender::Span<MLoopTri> loopTris = mesh->looptris();

  /* face_vertex_counts */
  face_vertex_counts = VtIntArray(tris_len, 3);

  /* face_vertex_indices */
  blender::Span<MLoop> loops = mesh->loops();
  face_vertex_indices.reserve(loopTris.size() * 3);
  for (MLoopTri lt : loopTris) {
    face_vertex_indices.push_back(loops[lt.tri[0]].v);
    face_vertex_indices.push_back(loops[lt.tri[1]].v);
    face_vertex_indices.push_back(loops[lt.tri[2]].v);
  }

  /* vertices */
  vertices.reserve(mesh->totvert);
  blender::Span<blender::float3> verts = mesh->vert_positions();
  for (blender::float3 v : verts) {
    vertices.push_back(GfVec3f(v.x, v.y, v.z));
  }

  /* normals */
  const float(*lnors)[3] = (float(*)[3])CustomData_get_layer(&mesh->ldata, CD_NORMAL);
  if (lnors) {
    normals.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      normals.push_back(GfVec3f(lnors[lt.tri[0]]));
      normals.push_back(GfVec3f(lnors[lt.tri[1]]));
      normals.push_back(GfVec3f(lnors[lt.tri[2]]));
    }
  }

  /* uvs*/
  const float(*luvs)[2] = (float(*)[2])CustomData_get_layer(&mesh->ldata, CD_PROP_FLOAT2);
  if (luvs) {
    uvs.reserve(loopTris.size() * 3);
    for (MLoopTri lt : loopTris) {
      uvs.push_back(GfVec2f(luvs[lt.tri[0]]));
      uvs.push_back(GfVec2f(luvs[lt.tri[1]]));
      uvs.push_back(GfVec2f(luvs[lt.tri[2]]));
    }
  }
}

void MeshData::insert_prim()
{
  if (face_vertex_counts.empty()) {
    return;
  }

  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().InsertRprim(HdPrimTypeTokens->mesh, scene_delegate, p_id);
  LOG(INFO) << "Add mesh: " << name() << " id=" << p_id.GetAsString();
}

void MeshData::remove_prim()
{
  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    return;
  }

  scene_delegate->GetRenderIndex().RemoveRprim(p_id);
  LOG(INFO) << "Remove mesh: " << name();
}

void MeshData::mark_prim_dirty(DirtyBits dirty_bits)
{
  SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    return;
  }

  HdDirtyBits bits = HdChangeTracker::Clean;
  switch (dirty_bits) {
    case DirtyBits::DirtyTransform:
      bits = HdChangeTracker::DirtyTransform;
      break;
    case DirtyBits::DirtyVisibility:
      bits = HdChangeTracker::DirtyVisibility;
      break;
    case DirtyBits::DirtyMaterial:
      bits = HdChangeTracker::DirtyMaterialId;
      break;
    case DirtyBits::AllDirty:
      bits = HdChangeTracker::AllDirty;
      break;
    default:
      break;
  }
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(p_id, bits);
  LOG(INFO) << "Update mesh: " << name() << " [" << (int)dirty_bits << "]";
}

}  // namespace blender::render::hydra
