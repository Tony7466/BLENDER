/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_lightprobe_types.h"
#include "overlay_next_extra_pass.hh"

namespace blender::draw::overlay {

static void probe_sync(const ObjectRef &ob_ref,
                       const select::ID select_id,
                       Resources &res,
                       const State &state,
                       ExtraInstancePass &pass,
                       ExtraInstanceData data)
{
  const LightProbe *probe = (LightProbe *)ob_ref.object->data;
  const bool show_clipping = (probe->flag & LIGHTPROBE_FLAG_SHOW_CLIP_DIST) != 0;
  const bool show_parallax = (probe->flag & LIGHTPROBE_FLAG_SHOW_PARALLAX) != 0;
  const bool show_influence = (probe->flag & LIGHTPROBE_FLAG_SHOW_INFLUENCE) != 0;
  const bool show_data = (ob_ref.object->base_flag & BASE_SELECTED) ||
                         res.selection_type != SelectionType::DISABLED;

  if (probe->type == LIGHTPROBE_TYPE_CUBE) {
    /* Pack render data into object matrix. */
    data.matrix[2][3] = show_clipping ? probe->clipsta : -1.0;
    data.matrix[3][3] = show_clipping ? probe->clipend : -1.0;

    pass.probe_cube.append(data, select_id);
    pass.groundline.append(float4(data.matrix.location()), select_id);

    if (show_influence) {
      float influence_start = probe->distinf * (1.0f - probe->falloff);
      float influence_end = probe->distinf;

      ExtraInstanceBuf &buf = (probe->attenuation_type == LIGHTPROBE_SHAPE_BOX) ? pass.cube :
                                                                                  pass.sphere;

      buf.append(data.with_size(influence_start), select_id);
      buf.append(data.with_size(influence_end), select_id);
    }

    if (show_parallax) {
      float radius = (probe->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) ? probe->distpar :
                                                                       probe->distinf;
      ExtraInstanceBuf &buf = (probe->parallax_type == LIGHTPROBE_SHAPE_BOX) ? pass.cube :
                                                                               pass.sphere;
      buf.append(data.with_size(radius), select_id);
    }
  }
  else if (probe->type == LIGHTPROBE_TYPE_GRID) {
    /* Pack render data into object matrix. */
    data.matrix[2][3] = show_clipping ? probe->clipsta : -1.0;
    data.matrix[3][3] = show_clipping ? probe->clipend : -1.0;
    pass.probe_grid.append(data, select_id);

    if (show_influence) {
      float influence_start = 1.0f + probe->distinf * (1.0f - probe->falloff);
      float influence_end = 1.0f + probe->distinf;

      pass.cube.append(data.with_size(influence_start), select_id);
      pass.cube.append(data.with_size(influence_end), select_id);
    }

    /* Data dots */
    if (show_data) {
      int3 resolution = int3(&probe->grid_resolution_x);
      int theme_id = res.object_wire_theme_id(ob_ref, state) == TH_ACTIVE ? 1 : 2;
      pass.probe_grid_dots.object_sync(ob_ref, select_id, resolution, theme_id, res, state);
    }
  }
  else if (probe->type == LIGHTPROBE_TYPE_PLANAR) {
    pass.probe_planar.append(data, select_id);

    if (res.selection_type != SelectionType::DISABLED && (probe->flag & LIGHTPROBE_FLAG_SHOW_DATA))
    {
      pass.quad.append(data, select_id);
    }

    if (show_influence) {
      data.matrix.z_axis() = math::normalize(data.matrix.z_axis()) * probe->distinf;
      pass.cube.append(data, select_id);
      data.matrix.z_axis() *= 1.0f - probe->falloff;
      pass.cube.append(data, select_id);
    }

    data.matrix.z_axis() = float3(0);
    pass.cube.append(data, select_id);

    data.matrix = float4x4(ob_ref.object->object_to_world);
    data.matrix.view<3, 3>() = math::normalize(data.matrix.view<3, 3>());
    pass.single_arrow.append(data.with_size(ob_ref.object->empty_drawsize), select_id);
  }
}

}  // namespace blender::draw::overlay
