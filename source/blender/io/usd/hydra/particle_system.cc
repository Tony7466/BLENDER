/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/tokens.h>

#include "BLI_string.h"

#include "BKE_customdata.h"
#include "BKE_material.h"

#include "BKE_curves.hh"

#include "DEG_depsgraph_query.hh"

#include "particle_system.h"
#include "hair.h"

#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

ParticleSystemData::ParticleSystemData(HydraSceneDelegate *scene_delegate,
                                   const Object *object,
                                   pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

std::unique_ptr<ParticleSystemData> ParticleSystemData::create(HydraSceneDelegate *scene_delegate,
                                                               const Object *object,
                                                               pxr::SdfPath const &prim_id,
                                                               ParticleSystem *particle_system)
{
  std::unique_ptr<ParticleSystemData> psys_data;
  if (particle_system->part) {
    switch (particle_system->part->type) {
      case PART_HAIR:
        psys_data = std::make_unique<HairData>(scene_delegate,
                                               object,
                                               prim_id,
                                               particle_system);
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
  if (psys_data) {
    psys_data->init();
  }  
  return psys_data;
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

pxr::VtValue ParticleSystemData::get_data(pxr::TfToken const & /*key*/) const
{
  return pxr::VtValue();
}

void ParticleSystemData::init() {}

void ParticleSystemData::insert() {}

void ParticleSystemData::remove() {}

void ParticleSystemData::update() {}

}  // namespace blender::io::hydra
