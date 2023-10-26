/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "curves.h"

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_curves.hh"
#include "BKE_customdata.h"
#include "BKE_material.h"
#include "BKE_particle.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"

#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

CurvesData::CurvesData(HydraSceneDelegate *scene_delegate,
                       const Object *object,
                       pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

void CurvesData::init()
{
  ID_LOGN(1, "");

  const Object *object = (const Object *)id;
  write_curves((const Curves *)object->data);
  write_transform();
  write_materials();
}

void CurvesData::insert()
{
  ID_LOGN(1, "");
  scene_delegate_->GetRenderIndex().InsertRprim(
      pxr::HdPrimTypeTokens->basisCurves, scene_delegate_, prim_id);
}

void CurvesData::remove()
{
  ID_LOG(1, "");
  scene_delegate_->GetRenderIndex().RemoveRprim(prim_id);
}

void CurvesData::update()
{
  const Object *object = (const Object *)id;
  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    bits = pxr::HdChangeTracker::AllDirty;
  }
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

  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(prim_id, bits);
  ID_LOGN(1, "");
}

pxr::VtValue CurvesData::get_data(pxr::TfToken const &key) const
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

pxr::SdfPath CurvesData::material_id() const
{
  if (!mat_data_) {
    return pxr::SdfPath();
  }
  return mat_data_->prim_id;
}

void CurvesData::available_materials(Set<pxr::SdfPath> &paths) const
{
  if (mat_data_ && !mat_data_->prim_id.IsEmpty()) {
    paths.add(mat_data_->prim_id);
  }
}

pxr::HdBasisCurvesTopology CurvesData::topology() const
{
  return pxr::HdBasisCurvesTopology(pxr::HdTokens->linear,
                                    pxr::TfToken(),
                                    pxr::HdTokens->nonperiodic,
                                    curve_vertex_counts_,
                                    pxr::VtIntArray());
}

pxr::HdPrimvarDescriptorVector CurvesData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationVertex) {
    if (!vertices_.empty()) {
      primvars.emplace_back(pxr::HdTokens->points, interpolation, pxr::HdPrimvarRoleTokens->point);
    }
    if (!widths_.empty()) {
      primvars.emplace_back(pxr::HdTokens->widths, interpolation, pxr::HdPrimvarRoleTokens->none);
    }
  }
  else if (interpolation == pxr::HdInterpolationConstant) {
    if (!uvs_.empty()) {
      primvars.emplace_back(pxr::HdPrimvarRoleTokens->textureCoordinate,
                            interpolation,
                            pxr::HdPrimvarRoleTokens->textureCoordinate);
    }
  }
  return primvars;
}

void CurvesData::write_materials()
{
  const Object *object = (const Object *)id;
  const Material *mat = nullptr;
  /* TODO: Using only first material. Add support for multi-material. */
  if (BKE_object_material_count_eval(object) > 0) {
    mat = BKE_object_material_get_eval(const_cast<Object *>(object), 0);
  }
  mat_data_ = get_or_create_material(mat);
}

void CurvesData::write_curves(const Curves *curves_id)
{
  const bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  curve_vertex_counts_.resize(curves.curves_num());
  offset_indices::copy_group_sizes(
      curves.points_by_curve(),
      curves.curves_range(),
      MutableSpan(curve_vertex_counts_.data(), curve_vertex_counts_.size()));

  const Span<float3> positions = curves.positions();
  vertices_.resize(curves.points_num());
  MutableSpan(vertices_.data(), vertices_.size()).copy_from(positions.cast<pxr::GfVec3f>());

  const VArray<float> radii = *curves.attributes().lookup_or_default<float>(
      "radius", ATTR_DOMAIN_POINT, 0.01f);
  widths_.resize(curves.points_num());
  for (const int i : curves.points_range()) {
    widths_[i] = radii[i] * 2.0f;
  }

  write_uv_maps(curves_id);
}

void CurvesData::write_uv_maps(const Curves *curves_id)
{
  const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  const Span<float2> surface_uv_coords = curves.surface_uv_coords();
  if (surface_uv_coords.is_empty()) {
    uvs_.clear();
    return;
  }

  uvs_.resize(curves.curves_num());
  MutableSpan(uvs_.data(), uvs_.size()).copy_from(surface_uv_coords.cast<pxr::GfVec2f>());
}

HairData::HairData(HydraSceneDelegate *scene_delegate,
                   const Object *object,
                   pxr::SdfPath const &prim_id,
                   ParticleSystem *particle_system)
    : CurvesData(scene_delegate, object, prim_id), particle_system(particle_system)
{
}

bool HairData::is_supported(const ParticleSystem *particle_system)
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

bool HairData::is_visible(HydraSceneDelegate *scene_delegate,
                          Object *object,
                          ParticleSystem *particle_system)
{
  const bool for_render = (DEG_get_mode(scene_delegate->depsgraph) == DAG_EVAL_RENDER);
  return psys_check_enabled(object, particle_system, for_render);
}

void HairData::init()
{
  ID_LOGN(1, "");
  /* NOTE: no need to write_transform here, since we already write actual position. */
  write_curves();
  write_materials();
  write_uv_maps();
}

void HairData::update()
{
  init();
  switch (particle_system->part->type) {
    case PART_HAIR:
      if (!scene_delegate_->GetRenderIndex().HasRprim(prim_id)) {
        insert();
      }
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
          prim_id, pxr::HdChangeTracker::AllDirty);
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

pxr::HdBasisCurvesTopology HairData::topology() const
{
  return pxr::HdBasisCurvesTopology(pxr::HdTokens->linear,
                                    pxr::TfToken(),
                                    pxr::HdTokens->nonperiodic,
                                    curve_vertex_counts_,
                                    pxr::VtIntArray());
}

void HairData::write_curves()
{
  ParticleCacheKey **cache = particle_system->pathcache;
  if (cache == nullptr) {
    return;
  }
  curve_vertex_counts_.clear();
  curve_vertex_counts_.reserve(particle_system->totpart);
  vertices_.clear();
  widths_.clear();

  ParticleCacheKey *strand;
  for (int strand_index = 0; strand_index < particle_system->totpart; ++strand_index) {
    strand = cache[strand_index];

    int point_count = strand->segments + 1;
    curve_vertex_counts_.push_back(point_count);

    for (int point_index = 0; point_index < point_count; ++point_index, ++strand) {
      vertices_.push_back(pxr::GfVec3f(strand->co));
      widths_.push_back(particle_system->part->rad_root * particle_system->part->rad_scale);
    }
  }
}

void HairData::write_uv_maps()
{
  uvs_.clear();
  uvs_.reserve(particle_system->totpart);
  if (particle_system->particles) {
    Object *object = (Object *)id;
    ParticleSystemModifierData *psmd = psys_get_modifier(object, particle_system);
    int num = ELEM(particle_system->particles->num_dmcache, DMCACHE_ISCHILD, DMCACHE_NOTFOUND) ?
                  particle_system->particles->num :
                  particle_system->particles->num_dmcache;

    float r_uv[2] = {0.0f, 0.0f};
    if (ELEM(psmd->psys->part->from, PART_FROM_FACE, PART_FROM_VOLUME) &&
        !ELEM(num, DMCACHE_NOTFOUND, DMCACHE_ISCHILD)) {
      const MFace *mface = static_cast<const MFace *>(
          CustomData_get_layer(&psmd->mesh_final->fdata_legacy, CD_MFACE));
      const MTFace *mtface = static_cast<const MTFace *>(
          CustomData_get_layer(&psmd->mesh_final->fdata_legacy, CD_MTFACE));

      
      if (mface && mtface) {
        mtface += num;
        psys_interpolate_uvs(mtface, mface->v4, particle_system->particles->fuv, r_uv);
      }
    }
    uvs_.push_back(pxr::GfVec2f(r_uv[0], r_uv[1]));
  }
}

}  // namespace blender::io::hydra
