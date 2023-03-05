/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "glog/logging.h"

#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_object.h"
#include "BKE_material.h"

#include "blenderSceneDelegate.h"
#include "mesh.h"
#include "../utils.h"

using namespace pxr;

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
  else if (key == HdInstancerTokens->instanceTransform) {
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
    if (!normals.empty()) {
      primvars.emplace_back(HdTokens->normals, interpolation, HdPrimvarRoleTokens->normal);
    }
    if (!uvs.empty()) {
      primvars.emplace_back(HdPrimvarRoleTokens->textureCoordinate, interpolation,
                            HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

HdPrimvarDescriptorVector MeshData::instancer_primvar_descriptors(HdInterpolation interpolation)
{
  HdPrimvarDescriptorVector primvars;
  if (interpolation == HdInterpolationInstance) {
    primvars.emplace_back(HdInstancerTokens->instanceTransform, interpolation,
                          HdPrimvarRoleTokens->none);
  }
  return primvars;
}

VtIntArray MeshData::instance_indices()
{
  VtIntArray ret(instances.size());
  for (size_t i = 0; i < ret.size(); ++i) {
    ret[i] = i;
  }
  return ret;
}

size_t MeshData::sample_instancer_transform(size_t maxSampleCount, float *sampleTimes, GfMatrix4d *sampleValues)
{
  *sampleTimes = 0.0f;
  *sampleValues = GfMatrix4d(1.0);
  return 1;
}

size_t MeshData::sample_instancer_primvar(TfToken const &key, size_t maxSampleCount, float *sampleTimes, VtValue *sampleValues)
{
  if (key == HdInstancerTokens->instanceTransform) {
    if (maxSampleCount > 0) {
      sampleTimes[0] = 0.0f;
      sampleValues[0] = instances;
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
    LOG(INFO) << "Add instancer: " << name() << " id=" << instancer_id.GetAsString();
  }
  if (instances.empty()) {
    // USD hides the prototype mesh when instancing in contrary to the Blender, so we must add it back implicitly
    instances.push_back(GfMatrix4d(1.0));
  }
  instances.push_back(transform().GetInverse() * gf_matrix_from_transform(dupli->mat));
  LOG(INFO) << "Add instance: " << instancer_id.GetAsString() << " " << dupli->random_id;
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
    /* Trying to insert prim */
    insert_prim();
    return;
  }

  if (face_vertex_counts.empty()) {
    /* Remove prim without faces */
    remove_prim();
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
