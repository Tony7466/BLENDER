/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <pxr/imaging/hd/tokens.h>

#include "BKE_customdata.h"
#include "BKE_particle.h"
#include "BKE_material.h"

#include "DNA_particle_types.h"
#include "DNA_modifier_types.h"
#include "DNA_mesh_types.h"

#include "hair.h"
#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

HairData::HairData(HydraSceneDelegate *scene_delegate,
                   const Object *object,
                   pxr::SdfPath const &prim_id,
                   ParticleSystem *particle_system)
    : ParticleSystemData(scene_delegate, object, prim_id), particle_system(particle_system)
{
}

void HairData::init()
{
  ID_LOGN(1, "");
  /* NOTE: no need to write_transform here, since we already write actual position. */
  write_hair();
  write_materials();
  write_uv_maps();
}

void HairData::insert()
{
  ID_LOGN(1, "");
  scene_delegate_->GetRenderIndex().InsertRprim( 
      pxr::HdPrimTypeTokens->basisCurves, scene_delegate_, prim_id);
}

void HairData::remove()
{
  ID_LOG(1, "");
  curve_vertex_counts_.clear();
  vertices_.clear();
  uvs_.clear();
  scene_delegate_->GetRenderIndex().RemoveRprim(prim_id);
}

void HairData::update()
{
  init();
  switch (particle_system->part->type) {
    case PART_HAIR:
      if (!scene_delegate_->GetRenderIndex().HasRprim(prim_id)) {
        insert();
      }
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(prim_id, pxr::HdChangeTracker::AllDirty);
      break;
    case PART_EMITTER:
    case PART_FLUID_FLIP:
    case PART_FLUID_SPRAY:
    case PART_FLUID_BUBBLE:
    case PART_FLUID_FOAM:
    case PART_FLUID_TRACER:
    case PART_FLUID_SPRAYFOAM:
    case PART_FLUID_SPRAYBUBBLE:
    case PART_FLUID_FOAMBUBBLE:
    case PART_FLUID_SPRAYFOAMBUBBLE:
      CLOG_WARN(LOG_HYDRA_SCENE, "Unsupported particle type: %d", particle_system->part->type);
      remove();
      break;
    default:
      BLI_assert_unreachable();
  }

  ID_LOGN(1, "");
}

pxr::VtValue HairData::get_data(pxr::TfToken const &key) const
{
  if (key == pxr::HdTokens->points) {
    return pxr::VtValue(vertices_);
  }
  else if (key == pxr::HdPrimvarRoleTokens->textureCoordinate) {
    return pxr::VtValue(uvs_);
  }
  else if (key == pxr::HdTokens->widths) {
    return pxr::VtValue(widths_);
  }
  return pxr::VtValue();
}

pxr::SdfPath HairData::material_id(pxr::SdfPath const &id) const
{
  if (!mat_data_) {
    return pxr::SdfPath();
  }
  return mat_data_->prim_id;
}

pxr::HdBasisCurvesTopology HairData::topology() const
{
  return pxr::HdBasisCurvesTopology(pxr::HdTokens->cubic,
                                    pxr::UsdGeomTokens->bspline,
                                    pxr::HdTokens->nonperiodic,
                                    curve_vertex_counts_,
                                    pxr::VtIntArray());
}

pxr::HdPrimvarDescriptorVector HairData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    if (!vertices_.empty()) {
      primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
    }
  }
  else if (interpolation == pxr::HdInterpolationConstant) {
    if (!widths_.empty()) {
      primvars.emplace_back(pxr::HdTokens->widths, interpolation, pxr::HdPrimvarRoleTokens->none);
    }
    if (!uvs_.empty()) {
      primvars.emplace_back(pxr::HdPrimvarRoleTokens->textureCoordinate,
                            interpolation,
                            pxr::HdPrimvarRoleTokens->textureCoordinate);
    }    
  }
  return primvars;
}

void HairData::write_materials()
{
  const Object *object = (const Object *)id;
  const Material *mat = nullptr;
  /* TODO: Using only first material. Add support for multi-material. */
  if (BKE_object_material_count_eval(object) > 0) {
    mat = BKE_object_material_get_eval(const_cast<Object *>(object), 0);
  }
  mat_data_ = get_or_create_material(mat);
}

void HairData::write_hair()
{
  ParticleCacheKey **cache = particle_system->pathcache;
  if (cache == nullptr) {
    return;
  }
  curve_vertex_counts_.clear();
  vertices_.clear();  

  curve_vertex_counts_.reserve(particle_system->totpart);
  vertices_.reserve(particle_system->totpart);  
  ParticleCacheKey *strand;
  for (int strand_index = 0; strand_index < particle_system->totpart; ++strand_index) {
    strand = cache[strand_index];

    int point_count = strand->segments + 1;
    curve_vertex_counts_.push_back(point_count);

    for (int point_index = 0; point_index < point_count; ++point_index, ++strand) {
      vertices_.push_back(pxr::GfVec3f(strand->co));
    }
  }
}

void HairData::write_uv_maps() {
  uvs_.clear();
  uvs_.reserve(particle_system->totpart);

  Object *object = (Object *)id;
  ParticleSystemModifierData *psmd = psys_get_modifier(object, particle_system);
  int num = ELEM(particle_system->particles->num_dmcache, DMCACHE_ISCHILD, DMCACHE_NOTFOUND) ?
                particle_system->particles->num :
                particle_system->particles->num_dmcache;

  const MFace *mface = static_cast<const MFace *>(
      CustomData_get_layer(&psmd->mesh_final->fdata_legacy, CD_MFACE));
  const MTFace *mtface = static_cast<const MTFace *>(
      CustomData_get_layer(&psmd->mesh_final->fdata_legacy, CD_MTFACE));

  float r_uv[2];
  if (mface && mtface) {
    mtface += num;
    psys_interpolate_uvs(mtface, mface->v4, particle_system->particles->fuv, r_uv);
  }
  for (int i = 0; i < particle_system->totpart; i++) {
    uvs_.push_back(pxr::GfVec2f(r_uv[0], r_uv[1]));
  }

}

}  // namespace blender::io::hydra
