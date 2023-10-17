/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "curves.h"

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "BLI_string.h"

#include "BKE_customdata.h"
#include "BKE_material.h"

#include "BKE_curves.hh"

#include "DEG_depsgraph_query.hh"

#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

ParticleSystemData::ParticleSystemData(HydraSceneDelegate *scene_delegate,
                                   const Object *object,
                                   pxr::SdfPath const &prim_id,
                                   ParticleSystem *particle_system)
    : ObjectData(scene_delegate, object, prim_id), particle_system(particle_system)
{
}

bool ParticleSystemData::is_supported(const ParticleSystem *particle_system)
{
  if (particle_system->part) {
    switch (particle_system->part->type) {
      case PART_HAIR:
        return true;
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
        return false;
        break;
      default:
        BLI_assert_unreachable();
    }
  }
  return false;
}

bool ParticleSystemData::is_visible(HydraSceneDelegate *scene_delegate,
                                    Object *object,
                                    ParticleSystem *particle_system,
                                    int object_mode)
{
  const bool for_render = (DEG_get_mode(scene_delegate->depsgraph) == DAG_EVAL_RENDER);
  return ObjectData::is_visible(scene_delegate, object) &&
         psys_check_enabled(object, particle_system, for_render);
}

void ParticleSystemData::init()
{
  ID_LOGN(1, "");
  /* NOTE: no need to write_transform here, since we already write actual position. */
  write_curves();
  write_materials();
}

void ParticleSystemData::insert()
{
  ID_LOGN(1, "");
  scene_delegate_->GetRenderIndex().InsertRprim( 
      pxr::HdPrimTypeTokens->basisCurves, scene_delegate_, prim_id);
}

void ParticleSystemData::remove()
{
  ID_LOG(1, "");
  curve_vertex_counts_.clear();
  vertices_.clear();
  scene_delegate_->GetRenderIndex().RemoveRprim(prim_id);
}

void ParticleSystemData::update()
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

pxr::VtValue ParticleSystemData::get_data(pxr::TfToken const &key) const
{
  if (key == pxr::HdTokens->points) {
    return pxr::VtValue(vertices_);
  }
  return pxr::VtValue();
}

pxr::SdfPath ParticleSystemData::material_id(pxr::SdfPath const &id) const
{
  if (!mat_data_) {
    return pxr::SdfPath();
  }
  return mat_data_->prim_id;
}

pxr::HdBasisCurvesTopology ParticleSystemData::topology() const
{
  return pxr::HdBasisCurvesTopology(pxr::HdTokens->cubic,
                                    pxr::UsdGeomTokens->bspline,
                                    pxr::HdTokens->nonperiodic,
                                    curve_vertex_counts_,
                                    pxr::VtIntArray());
}

pxr::HdPrimvarDescriptorVector ParticleSystemData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    if (!vertices_.empty()) {
      primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
    }
  }
  return primvars;
}

void ParticleSystemData::write_materials()
{
  const Object *object = (const Object *)id;
  const Material *mat = nullptr;
  /* TODO: Using only first material. Add support for multi-material. */
  if (BKE_object_material_count_eval(object) > 0) {
    mat = BKE_object_material_get_eval(const_cast<Object *>(object), 0);
  }
  mat_data_ = get_or_create_material(mat);
}

void ParticleSystemData::write_curves() {
  switch (particle_system->part->type) {
    case PART_HAIR:
      write_hair();
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
      break;
    default:
      BLI_assert_unreachable();
  }
}

void ParticleSystemData::write_hair()
{
  ParticleCacheKey **cache = particle_system->pathcache;
  if (cache == nullptr) {
    return;
  }
  vertices_.clear();
  curve_vertex_counts_.clear();

  vertices_.reserve(particle_system->totpart);
  curve_vertex_counts_.reserve(particle_system->totpart);
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

}  // namespace blender::io::hydra
