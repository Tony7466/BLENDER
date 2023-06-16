/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_mball.h"
#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class BoundPassesBase : public OverlayPasses {

  ExtraInstanceBuf cube = extra_buf("cube", shapes.empty_cube);
  ExtraInstanceBuf sphere = extra_buf("sphere", shapes.empty_sphere);
  ExtraInstanceBuf cone = extra_buf("cone", shapes.empty_cone);
  ExtraInstanceBuf cylinder = extra_buf("cylinder", shapes.empty_cylinder);
  ExtraInstanceBuf capsule_body = extra_buf("capsule_body", shapes.empty_capsule_body);
  ExtraInstanceBuf capsule_cap = extra_buf("capsule_cap", shapes.empty_capsule_cap);

 public:
  BoundPassesBase(const char *name,
                  SelectionType selection_type,
                  const ShapeCache &shapes,
                  const GlobalsUboStorage &theme_colors,
                  bool in_front)
      : OverlayPasses(name, selection_type, shapes, theme_colors, in_front){};

  void bounds_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources &res,
                   const State &state,
                   char boundtype,
                   bool around_origin)
  {
    ExtraInstanceData data(ob_ref.object, res.object_wire_color(ob_ref, state));

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
      cube.append(data.with_matrix(data.matrix * mat), select_id);
    }
    else if (boundtype == OB_BOUND_SPHERE) {
      size = float3(std::max({size.x, size.y, size.z}));
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      sphere.append(data.with_matrix(data.matrix * mat), select_id);
    }
    else if (boundtype == OB_BOUND_CYLINDER) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      cylinder.append(data.with_matrix(data.matrix * mat), select_id);
    }
    else if (boundtype == OB_BOUND_CONE) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      /* Cone batch has base at 0 and is pointing towards +Y. */
      std::swap(mat[1], mat[2]);
      mat.location().z -= size.z;
      cone.append(data.with_matrix(data.matrix * mat), select_id);
    }
    else if (boundtype == OB_BOUND_CAPSULE) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(float3(size.x));
      mat.location() = center;

      mat.location().z = center.z + std::max(0.0f, size.z - size.x);
      capsule_cap.append(data.with_matrix(data.matrix * mat), select_id);

      mat.location().z = center.z - std::max(0.0f, size.z - size.x);
      mat.z_axis() *= -1.0f;
      capsule_cap.append(data.with_matrix(data.matrix * mat), select_id);

      mat.z_axis().z = std::max(0.0f, (size.z - size.x) * 2.0f);
      capsule_body.append(data.with_matrix(data.matrix * mat), select_id);
    }
    else {
      BLI_assert_unreachable();
    }
  }
};

class BoundPasses : public BoundPassesBase {

 public:
  BoundPasses(SelectionType selection_type,
              const ShapeCache &shapes,
              const GlobalsUboStorage &theme_colors,
              bool in_front)
      : BoundPassesBase("Bounds", selection_type, shapes, theme_colors, in_front){};

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) final override
  {
    Object *ob = ob_ref.object;
    const bool from_dupli = ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI);
    const bool has_bounds = !ELEM(
        ob->type, OB_LAMP, OB_CAMERA, OB_EMPTY, OB_SPEAKER, OB_LIGHTPROBE);
    const bool draw_bounds = ob->dt == OB_BOUNDBOX || ob->dtx & OB_DRAWBOUNDOX;
    if (from_dupli || !has_bounds || !draw_bounds) {
      return;
    }

    bounds_sync(ob_ref, select_id, res, state, ob->boundtype, false);
  }
};

using Bound = OverlayType<BoundPasses>;

}  // namespace blender::draw::overlay
