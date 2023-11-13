/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/imaging/hd/tokens.h>

#include "BLI_array_utils.hh"
#include "BLI_string.h"

#include "BKE_attribute.h"
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"

#include "hydra_scene_delegate.h"
#include "mesh.h"

PXR_NAMESPACE_OPEN_SCOPE
TF_DEFINE_PRIVATE_TOKENS(tokens_, (st));
PXR_NAMESPACE_CLOSE_SCOPE

namespace blender::io::hydra {

MeshData::MeshData(HydraSceneDelegate *scene_delegate,
                   const Object *object,
                   pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

void MeshData::init()
{
  ID_LOGN(1, "");

  Object *object = (Object *)id;
  Mesh *mesh = BKE_object_to_mesh(nullptr, object, false);
  if (mesh) {
    write_submeshes(mesh);
  }
  BKE_object_to_mesh_clear(object);

  write_transform();
  write_materials();
}

void MeshData::insert()
{
  ID_LOGN(1, "");
  update_prims();
}

void MeshData::remove()
{
  ID_LOG(1, "");
  submeshes_.clear();
  update_prims();
}

void MeshData::update()
{
  Object *object = (Object *)id;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    update_prims();
    return;
  }

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  if (id->recalc & ID_RECALC_SHADING) {
    write_materials();
    bits |= pxr::HdChangeTracker::DirtyMaterialId | pxr::HdChangeTracker::DirtyDoubleSided;
  }
  if (id->recalc & ID_RECALC_TRANSFORM) {
    write_transform();
    bits |= pxr::HdChangeTracker::DirtyTransform;
  }

  if (bits == pxr::HdChangeTracker::Clean) {
    return;
  }

  for (int i = 0; i < submeshes_.size(); ++i) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(submesh_prim_id(i), bits);
    ID_LOGN(1, "%d", i);
  }
}

pxr::VtValue MeshData::get_data(pxr::TfToken const & /*key*/) const
{
  return pxr::VtValue();
}

pxr::VtValue MeshData::get_data(pxr::SdfPath const &id, pxr::TfToken const &key) const
{
  if (key == pxr::HdTokens->normals) {
    return pxr::VtValue(submesh(id).normals);
  }
  if (key == pxr::tokens_->st) {
    return pxr::VtValue(submesh(id).uvs);
  }
  if (key == pxr::HdTokens->points) {
    return pxr::VtValue(submesh(id).vertices);
  }

  return get_data(key);
}

pxr::SdfPath MeshData::material_id(pxr::SdfPath const &id) const
{
  const SubMesh &sm = submesh(id);
  if (!sm.mat_data) {
    return pxr::SdfPath();
  }
  return sm.mat_data->prim_id;
}

void MeshData::available_materials(Set<pxr::SdfPath> &paths) const
{
  for (auto &sm : submeshes_) {
    if (sm.mat_data && !sm.mat_data->prim_id.IsEmpty()) {
      paths.add(sm.mat_data->prim_id);
    }
  }
}

pxr::HdMeshTopology MeshData::topology(pxr::SdfPath const &id) const
{
  const SubMesh &sm = submesh(id);
  return pxr::HdMeshTopology(pxr::PxOsdOpenSubdivTokens->none,
                             pxr::HdTokens->rightHanded,
                             sm.face_vertex_counts,
                             sm.face_vertex_indices);
}

pxr::HdPrimvarDescriptorVector MeshData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
  }
  else if (interpolation == pxr::HdInterpolationFaceVarying) {
    if (!submeshes_[0].normals.empty()) {
      primvars.emplace_back(
          pxr::HdTokens->normals, interpolation, pxr::HdPrimvarRoleTokens->normal);
    }
    if (!submeshes_[0].uvs.empty()) {
      primvars.emplace_back(
          pxr::tokens_->st, interpolation, pxr::HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

pxr::HdCullStyle MeshData::cull_style(pxr::SdfPath const &id) const
{
  const SubMesh &sm = submesh(id);
  if (sm.mat_data) {
    return sm.mat_data->cull_style();
  }
  return pxr::HdCullStyle::HdCullStyleNothing;
}

bool MeshData::double_sided(pxr::SdfPath const &id) const
{
  const SubMesh &sm = submesh(id);
  if (sm.mat_data) {
    return sm.mat_data->double_sided;
  }
  return true;
}

void MeshData::update_double_sided(MaterialData *mat_data)
{
  for (int i = 0; i < submeshes_.size(); ++i) {
    if (submeshes_[i].mat_data == mat_data) {
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
          submesh_prim_id(i),
          pxr::HdChangeTracker::DirtyDoubleSided | pxr::HdChangeTracker::DirtyCullStyle);
      ID_LOGN(1, "%d", i);
    }
  }
}

pxr::SdfPathVector MeshData::submesh_paths() const
{
  pxr::SdfPathVector ret;
  for (int i = 0; i < submeshes_.size(); ++i) {
    ret.push_back(submesh_prim_id(i));
  }
  return ret;
}

void MeshData::write_materials()
{
  const Object *object = (const Object *)id;
  for (int i = 0; i < submeshes_.size(); ++i) {
    SubMesh &m = submeshes_[i];
    const Material *mat = BKE_object_material_get_eval(const_cast<Object *>(object),
                                                       m.mat_index + 1);
    m.mat_data = get_or_create_material(mat);
  }
}

pxr::SdfPath MeshData::submesh_prim_id(int index) const
{
  char name[16];
  SNPRINTF(name, "SM_%04d", index);
  return prim_id.AppendElementString(name);
}

const MeshData::SubMesh &MeshData::submesh(pxr::SdfPath const &id) const
{
  int index;
  sscanf(id.GetName().c_str(), "SM_%d", &index);
  return submeshes_[index];
}

static void copy_submesh(const Span<float3> vert_positions,
                         const Span<int> corner_verts,
                         const Span<MLoopTri> looptris,
                         const Span<float3> corner_normals,
                         const Span<float2> uv_map,
                         const IndexMask &triangles,
                         MeshData::SubMesh &sm)
{

  sm.face_vertex_indices.resize(triangles.size() * 3);
  triangles.foreach_index(GrainSize(1024), [&](const int src, const int dst) {
    const MLoopTri &tri = looptris[src];
    sm.face_vertex_indices[dst * 3 + 0] = corner_verts[tri.tri[0]];
    sm.face_vertex_indices[dst * 3 + 1] = corner_verts[tri.tri[1]];
    sm.face_vertex_indices[dst * 3 + 2] = corner_verts[tri.tri[2]];
  });

  sm.vertices.resize(vert_count);
  array_utils::gather(vert_positions,
                      {sm.face_vertex_indices.data(), sm.face_vertex_indices.size()},
                      Span<pxr::GfVec3f>(sm.vertices.data(), sm.vertices.size()).cast<float3>());

  {
    /* Compress vertex indices to be contiguous so it's only necessary to copy used vertices. */
    Array<int> index_map(vert_positions.size(), 0);
    int vert_count = 0;
    const auto remap_vert = [&](const int vert) {
      if (index_map[vert] != 0) {
        index_map[vert] = vert_count;
      }
      return index_map[vert];
    };
    for (int &index : sm.face_vertex_indices) {
      index = remap_vert(index);
    }
  }

  sm.face_vertex_counts.resize(triangles.size());
  std::fill(sm.face_vertex_counts.begin(), sm.face_vertex_counts.end(), 3);

  sm.normals.resize(triangles.size() * 3);
  triangles.foreach_index(GrainSize(1024), [&](const int src, const int dst) {
    const MLoopTri &tri = looptris[src];
    sm.normals[dst * 3 + 0] = pxr::GfVec3f(&corner_normals[tri.tri[0]].x);
    sm.normals[dst * 3 + 1] = pxr::GfVec3f(&corner_normals[tri.tri[1]].x);
    sm.normals[dst * 3 + 2] = pxr::GfVec3f(&corner_normals[tri.tri[2]].x);
  });

  if (!uv_map.is_empty()) {
    sm.uvs.resize(triangles.size() * 3);
    triangles.foreach_index(GrainSize(1024), [&](const int src, const int dst) {
      const MLoopTri &tri = looptris[src];
      sm.uvs[dst * 3 + 0] = pxr::GfVec2f(&uv_map[tri.tri[0]].x);
      sm.uvs[dst * 3 + 1] = pxr::GfVec2f(&uv_map[tri.tri[1]].x);
      sm.uvs[dst * 3 + 2] = pxr::GfVec2f(&uv_map[tri.tri[2]].x);
    });
  }
}

void MeshData::write_submeshes(const Mesh *mesh)
{
  const int mat_count = BKE_object_material_count_eval(reinterpret_cast<const Object *>(id));
  submeshes_.reinitialize(mat_count > 0 ? mat_count : 1);
  for (const int i : submeshes_.index_range()) {
    submeshes_[i].mat_index = i;
  }

  const Span<float3> vert_positions = mesh->vert_positions();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<MLoopTri> looptris = mesh->looptris();

  Span<float3> corner_normals;
  if (mesh->normals_domain() == blender::bke::MeshNormalDomain::Corner) {
    corner_normals = mesh->corner_normals();
  }

  const float2 *uv_map = static_cast<const float2 *>(
      CustomData_get_layer(&mesh->loop_data, CD_PROP_FLOAT2));

  const int *material_indices = BKE_mesh_material_indices(mesh);
  if (!material_indices) {
    copy_submesh(vert_positions,
                 corner_verts,
                 looptris,
                 corner_normals,
                 uv_map ? Span<float2>(uv_map, mesh->totloop) : Span<float2>(),
                 looptris.index_range(),
                 submeshes_.first()) return;
  }

  const Span<int> looptri_faces = mesh->looptri_faces();

  IndexMaskMemory memory;
  Array<IndexMask> triangles_by_material(submeshes_.size());
  IndexMask::from_groups(
      looptris.index_range(),
      memory,
      [&](const int i) { return material_indices[looptri_faces[i]]; },
      triangles_by_material);

  threading::parallel_for(submeshes_.index_range(), 1, [&](const IndexRange range) {
    for (const int i : range) {
      copy_submesh(mesh,
                   corner_verts,
                   corner_normals,
                   uv_map ? Span<float2>(uv_map, mesh->totloop) : Span<float2>(),
                   triangles_by_material[i],
                   submeshes_[i]);
    }
  });

  /* Remove submeshes without faces */
  submeshes_.remove_if([](const SubMesh &submesh) { return submesh.face_vertex_counts.empty(); });
}

void MeshData::update_prims()
{
  auto &render_index = scene_delegate_->GetRenderIndex();
  int i;
  for (i = 0; i < submeshes_.size(); ++i) {
    pxr::SdfPath p = submesh_prim_id(i);
    if (i < submeshes_count_) {
      render_index.GetChangeTracker().MarkRprimDirty(p, pxr::HdChangeTracker::AllDirty);
      ID_LOGN(1, "Update %d", i);
    }
    else {
      render_index.InsertRprim(pxr::HdPrimTypeTokens->mesh, scene_delegate_, p);
      ID_LOGN(1, "Insert %d", i);
    }
  }
  for (; i < submeshes_count_; ++i) {
    render_index.RemoveRprim(submesh_prim_id(i));
    ID_LOG(1, "Remove %d", i);
  }
  submeshes_count_ = submeshes_.size();
}

}  // namespace blender::io::hydra
