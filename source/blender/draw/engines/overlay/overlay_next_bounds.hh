/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_mball.h"
#include "overlay_next_extra_pass.hh"

namespace blender::draw::overlay {

static void bounds_sync_base(const ObjectRef &ob_ref,
                             const select::ID select_id,
                             ExtraInstancePass &pass,
                             ExtraInstanceData data,
                             char boundtype,
                             bool around_origin)
{
  Object *ob = ob_ref.object;

  if (ObjectType(ob->type) == OB_MBALL && !BKE_mball_is_basis(ob)) {
    return;
  }

  const BoundBox *bb = BKE_object_boundbox_get(ob);
  BoundBox bb_local;
  if (bb == nullptr) {
    const float3 min = float3(-1.0f);
    const float3 max = float3(1.0f);
    BKE_boundbox_init_from_minmax(&bb_local, min, max);
    bb = &bb_local;
  }

  float3 center = float3(0);
  if (!around_origin) {
    BKE_boundbox_calc_center_aabb(bb, center);
  }

  float3 size;
  BKE_boundbox_calc_size_aabb(bb, size);

  if (boundtype == OB_BOUND_BOX) {
    float4x4 mat = math::from_scale<float4x4>(size);
    mat.location() = center;
    pass.cube.append(data.with_matrix(data.matrix * mat), select_id);
  }
  else if (boundtype == OB_BOUND_SPHERE) {
    size = float3(std::max({size.x, size.y, size.z}));
    float4x4 mat = math::from_scale<float4x4>(size);
    mat.location() = center;
    pass.sphere.append(data.with_matrix(data.matrix * mat), select_id);
  }
  else if (boundtype == OB_BOUND_CYLINDER) {
    size.x = size.y = std::max(size.x, size.y);
    float4x4 mat = math::from_scale<float4x4>(size);
    mat.location() = center;
    pass.cylinder.append(data.with_matrix(data.matrix * mat), select_id);
  }
  else if (boundtype == OB_BOUND_CONE) {
    size.x = size.y = std::max(size.x, size.y);
    float4x4 mat = math::from_scale<float4x4>(size);
    mat.location() = center;
    /* Cone batch has base at 0 and is pointing towards +Y. */
    std::swap(mat[1], mat[2]);
    mat.location().z -= size.z;
    pass.cone.append(data.with_matrix(data.matrix * mat), select_id);
  }
  else if (boundtype == OB_BOUND_CAPSULE) {
    size.x = size.y = std::max(size.x, size.y);
    float4x4 mat = math::from_scale<float4x4>(float3(size.x));
    mat.location() = center;

    mat.location().z = center.z + std::max(0.0f, size.z - size.x);
    pass.capsule_cap.append(data.with_matrix(data.matrix * mat), select_id);

    mat.location().z = center.z - std::max(0.0f, size.z - size.x);
    mat.z_axis() *= -1.0f;
    pass.capsule_cap.append(data.with_matrix(data.matrix * mat), select_id);

    mat.z_axis().z = std::max(0.0f, (size.z - size.x) * 2.0f);
    pass.capsule_body.append(data.with_matrix(data.matrix * mat), select_id);
  }
}

static void bounds_sync(const ObjectRef &ob_ref,
                        const select::ID select_id,
                        Resources & /*res*/,
                        const State & /*state*/,
                        ExtraInstancePass &pass,
                        ExtraInstanceData data)
{
  Object *ob = ob_ref.object;
  bounds_sync_base(ob_ref, select_id, pass, data, ob->boundtype, false);
}

}  // namespace blender::draw::overlay
