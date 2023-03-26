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
    : ObjectData(scene_delegate, object)
{
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
}

pxr::VtValue MeshData::get_data(pxr::TfToken const &key)
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

Material *MeshData::material()
{
  Object *object = (Object *)id;
  if (BKE_object_material_count_eval(object) == 0) {
    return nullptr;
  }
  return BKE_object_material_get_eval(object, object->actcol);
}

pxr::HdMeshTopology MeshData::mesh_topology()
{
  return pxr::HdMeshTopology(pxr::PxOsdOpenSubdivTokens->catmullClark,
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

pxr::HdPrimvarDescriptorVector MeshData::instancer_primvar_descriptors(
    pxr::HdInterpolation interpolation)
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationInstance) {
    primvars.emplace_back(
        pxr::HdInstancerTokens->instanceTransform, interpolation, pxr::HdPrimvarRoleTokens->none);
  }
  return primvars;
}

pxr::VtIntArray MeshData::instance_indices()
{
  pxr::VtIntArray ret(instances.size());
  for (size_t i = 0; i < ret.size(); ++i) {
    ret[i] = i;
  }
  return ret;
}

size_t MeshData::sample_instancer_transform(size_t max_sample_count,
                                            float *sample_times,
                                            pxr::GfMatrix4d *sample_values)
{
  *sample_times = 0.0f;
  *sample_values = pxr::GfMatrix4d(1.0);
  return 1;
}

size_t MeshData::sample_instancer_primvar(pxr::TfToken const &key,
                                          size_t max_sample_count,
                                          float *sample_times,
                                          pxr::VtValue *sample_values)
{
  if (key == pxr::HdInstancerTokens->instanceTransform) {
    if (max_sample_count > 0) {
      sample_times[0] = 0.0f;
      sample_values[0] = instances;
      return 1;
    }
  }
  return 0;
}

void MeshData::add_instance(DupliObject *dupli)
{
  if (instancer_id.IsEmpty()) {
    instancer_id = prim_id(scene_delegate, (Object *)id).AppendElementString("Instancer");
    scene_delegate->GetRenderIndex().InsertInstancer(scene_delegate, instancer_id);
    CLOG_INFO(LOG_BSD, 2, "Instancer: %s, id=%s", name().c_str(), instancer_id.GetText());
  }
  if (instances.empty()) {
    // USD hides the prototype mesh when instancing in contrary to the Blender, so we must add it
    // back implicitly
    instances.push_back(pxr::GfMatrix4d(1.0));
  }
  instances.push_back(transform().GetInverse() * gf_matrix_from_transform(dupli->mat));
  CLOG_INFO(LOG_BSD, 2, "%s - %d", instancer_id.GetText(), dupli->random_id);
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

void MeshData::insert_prim()
{
  if (face_vertex_counts.empty()) {
    return;
  }

  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  scene_delegate->GetRenderIndex().InsertRprim(pxr::HdPrimTypeTokens->mesh, scene_delegate, p_id);
  CLOG_INFO(LOG_BSD, 2, "Add: %s id=%s", name().c_str(), p_id.GetString().c_str());
}

void MeshData::remove_prim()
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    return;
  }

  scene_delegate->GetRenderIndex().RemoveRprim(p_id);
  CLOG_INFO(LOG_BSD, 2, "Remove: %s", name().c_str());
}

void MeshData::mark_prim_dirty(DirtyBits dirty_bits)
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Object *)id);
  if (!scene_delegate->GetRenderIndex().HasRprim(p_id)) {
    /* Trying to insert prim */
    insert_prim();
    return;
  }

  if (face_vertex_counts.empty()) {
    /* Remove prim without faces */
    remove_prim();
    return;
  }

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  switch (dirty_bits) {
    case DirtyBits::DIRTY_TRANSFORM:
      bits = pxr::HdChangeTracker::DirtyTransform;
      break;
    case DirtyBits::DIRTY_VISIBILITY:
      bits = pxr::HdChangeTracker::DirtyVisibility;
      break;
    case DirtyBits::DIRTY_MATERIAL:
      bits = pxr::HdChangeTracker::DirtyMaterialId;
      break;
    case DirtyBits::ALL_DIRTY:
      bits = pxr::HdChangeTracker::AllDirty;
      break;
    default:
      break;
  }
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(p_id, bits);
  CLOG_INFO(LOG_BSD, 2, "Update: [%d] %s", dirty_bits, name().c_str());
}

}  // namespace blender::render::hydra
