/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_lightprobe_types.h"
#include "overlay_next_extra_passes.hh"

namespace blender::draw::overlay {

static void probe_sync(const ObjectRef &ob_ref,
                       const select::ID select_id,
                       Resources &res,
                       const State &state,
                       ExtraInstancePasses &passes,
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

    passes.probe_cube.append(data, select_id);
    passes.groundline.append(float4(data.matrix.location()), select_id);

    if (show_influence) {
      float influence_start = probe->distinf * (1.0f - probe->falloff);
      float influence_end = probe->distinf;

      ExtraInstanceBuf &buf = (probe->attenuation_type == LIGHTPROBE_SHAPE_BOX) ? passes.cube :
                                                                                  passes.sphere;

      buf.append(data.with_size(influence_start), select_id);
      buf.append(data.with_size(influence_end), select_id);
    }

    if (show_parallax) {
      float radius = (probe->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) ? probe->distpar :
                                                                       probe->distinf;
      ExtraInstanceBuf &buf = (probe->parallax_type == LIGHTPROBE_SHAPE_BOX) ? passes.cube :
                                                                               passes.sphere;
      buf.append(data.with_size(radius), select_id);
    }
  }
  else if (probe->type == LIGHTPROBE_TYPE_GRID) {
    /* Pack render data into object matrix. */
    data.matrix[2][3] = show_clipping ? probe->clipsta : -1.0;
    data.matrix[3][3] = show_clipping ? probe->clipend : -1.0;
    passes.probe_grid.append(data, select_id);

    if (show_influence) {
      float influence_start = 1.0f + probe->distinf * (1.0f - probe->falloff);
      float influence_end = 1.0f + probe->distinf;

      passes.cube.append(data.with_size(influence_start), select_id);
      passes.cube.append(data.with_size(influence_end), select_id);
    }

    /* Data dots */
    if (show_data) {
      /* Pack render data into object matrix. */
      data.matrix[0][3] = probe->grid_resolution_x;
      data.matrix[1][3] = probe->grid_resolution_y;
      data.matrix[2][3] = probe->grid_resolution_z;

      /* Put theme id in matrix. */
      if (res.object_wire_theme_id(ob_ref, state) == TH_ACTIVE) {
        data.matrix[3][3] = 1.0;
      }
      else /* TH_SELECT */ {
        data.matrix[3][3] = 2.0;
      }

      uint cell_count = probe->grid_resolution_x * probe->grid_resolution_y *
                        probe->grid_resolution_z;
#if 0
      /* TODO(Miguel Pozo) */
      DRWShadingGroup *grp = DRW_shgroup_create_sub(vedata.stl->pd->extra_grid_grp);
      DRW_shgroup_uniform_mat4_copy(grp, "gridModelMatrix", data.matrix);
      DRW_shgroup_call_procedural_points(grp, nullptr, cell_count);
#endif
    }
  }
  else if (probe->type == LIGHTPROBE_TYPE_PLANAR) {
    passes.probe_planar.append(data, select_id);

    if (res.selection_type != SelectionType::DISABLED && (probe->flag & LIGHTPROBE_FLAG_SHOW_DATA))
    {
      passes.quad.append(data, select_id);
    }

    if (show_influence) {
      data.matrix.z_axis() = math::normalize(data.matrix.z_axis()) * probe->distinf;
      passes.cube.append(data, select_id);
      data.matrix.z_axis() *= 1.0f - probe->falloff;
      passes.cube.append(data, select_id);
    }

    data.matrix.z_axis() = float3(0);
    passes.cube.append(data, select_id);

    data.matrix = float4x4(ob_ref.object->object_to_world);
    data.matrix.view<3, 3>() = math::normalize(data.matrix.view<3, 3>());
    passes.single_arrow.append(data.with_size(ob_ref.object->empty_drawsize), select_id);
  }
}

}  // namespace blender::draw::overlay
