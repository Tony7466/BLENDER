/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_anim_path.h"
#include "DNA_curve_types.h"
#include "DNA_object_force_types.h"
#include "overlay_next_extra_passes.hh"

namespace blender::draw::overlay {

static void force_field_sync(const ObjectRef &ob_ref,
                             const select::ID select_id,
                             Resources &res,
                             const State &state,
                             ExtraInstancePasses &passes,
                             ExtraInstanceData data)
{
  /* Pack render data into object matrix. */
  float &size_x = data.matrix[0].w;
  float &size_y = data.matrix[1].w;
  float &size_z = data.matrix[2].w;

  Object *ob = ob_ref.object;
  PartDeflect *pd = ob->pd;
  Curve *cu = (ob->type == OB_CURVES_LEGACY) ? static_cast<Curve *>(ob->data) : nullptr;

  data.color = res.object_background_blend_color(ob_ref, state);
  size_x = size_y = size_z = ob->empty_drawsize;

  switch (pd->forcefield) {
    case PFIELD_FORCE:
      passes.field_force.append(data, select_id);
      break;
    case PFIELD_WIND:
      size_z = pd->f_strength;
      passes.field_wind.append(data, select_id);
      break;
    case PFIELD_VORTEX:
      size_y = (pd->f_strength < 0.0f) ? -size_y : size_y;
      passes.field_vortex.append(data, select_id);
      break;
    case PFIELD_GUIDE:
      if (cu && (cu->flag & CU_PATH) && ob->runtime.curve_cache->anim_path_accum_length) {
        float4x4 matrix = data.matrix;
        float4 position;

        size_x = size_y = size_z = pd->f_strength;

        BKE_where_on_path(ob, 0.0f, position, nullptr, nullptr, nullptr, nullptr);
        matrix.location() = data.matrix.location() + position.xyz();
        passes.field_curve.append(data.with_matrix(matrix), select_id);

        BKE_where_on_path(ob, 1.0f, position, nullptr, nullptr, nullptr, nullptr);
        matrix.location() = data.matrix.location() + position.xyz();
        passes.field_sphere_limit.append(data.with_matrix(matrix), select_id);
      }
      break;
  }

  if (pd->falloff == PFIELD_FALL_TUBE) {
    if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
      size_z = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;
      size_x = (pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f;
      size_y = size_x;
      passes.field_tube_limit.append(data, select_id);
    }
    if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
      size_z = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;
      size_x = (pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f;
      size_y = size_x;
      passes.field_tube_limit.append(data, select_id);
    }
  }
  else if (pd->falloff == PFIELD_FALL_CONE) {
    if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
      float radius = DEG2RADF((pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f);
      float distance = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;
      size_x = distance * math::sin(radius);
      size_z = distance * math::cos(radius);
      size_y = size_x;
      passes.field_cone_limit.append(data, select_id);
    }
    if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
      float radius = DEG2RADF((pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f);
      float distance = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;
      size_x = distance * math::sin(radius);
      size_z = distance * math::cos(radius);
      size_y = size_x;
      passes.field_cone_limit.append(data, select_id);
    }
  }
  else if (pd->falloff == PFIELD_FALL_SPHERE) {
    if (pd->flag & PFIELD_USEMAX) {
      size_x = size_y = size_z = pd->maxdist;
      passes.field_sphere_limit.append(data, select_id);
    }
    if (pd->flag & PFIELD_USEMIN) {
      size_x = size_y = size_z = pd->mindist;
      passes.field_sphere_limit.append(data, select_id);
    }
  }
}

}  // namespace blender::draw::overlay
