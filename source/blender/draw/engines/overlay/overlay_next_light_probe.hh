/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_lightprobe_types.h"
#include "overlay_next_empty.hh"
#include "overlay_next_light.hh"

namespace blender::draw::overlay {

class DotsGridPass {
  PassMain ps_ = {"Probe Grid"};

 public:
  void begin_sync(Resources &res, const State &state)
  {
    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_PROGRAM_POINT_SIZE | state.clipping_state);
    ps_.shader_set(res.shaders.extra_grid.get());
    /* TODO(Miguel Pozo): Selection doesn't work. */
    res.select_bind(ps_);
    /* TODO: Fixed index. */
    ps_.bind_ubo("globalsBlock", &res.globals_buf);
    ps_.bind_texture("depthBuffer", &res.depth_tx);
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   int3 grid_resolution,
                   int theme_id,
                   Resources & /*res*/,
                   const State &state)
  {
    float4x4 matrix = float4x4(ob_ref.object->object_to_world);
    /* Pack render data into object matrix. */
    matrix[0][3] = grid_resolution.x;
    matrix[1][3] = grid_resolution.y;
    matrix[2][3] = grid_resolution.z;
    matrix[3][3] = theme_id;

    uint cell_count = grid_resolution.x * grid_resolution.y * grid_resolution.z;

    ResourceHandle res_handle = state.manager->resource_handle(matrix);
    ps_.draw_procedural(GPU_PRIM_POINTS, 1, cell_count, 0, res_handle, select_id.get());
  }

  void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(ps_, view);
  }
};

class LightProbePasses : public OverlayPasses {

  DotsGridPass dots_grid;

  GroundLineInstanceBuf groundline = {"groundline", shapes.groundline.get(), selection_type};

  ExtraInstanceBuf probe_cube = extra_buf("probe_cube", shapes.probe_cube);
  ExtraInstanceBuf probe_grid = extra_buf("probe_grid", shapes.probe_grid);
  ExtraInstanceBuf probe_planar = extra_buf("probe_planar", shapes.probe_planar);

  ExtraInstanceBuf single_arrow = extra_buf("single_arrow", shapes.single_arrow);
  ExtraInstanceBuf quad = extra_buf("quad", shapes.quad);
  ExtraInstanceBuf cube = extra_buf("cube", shapes.empty_cube);
  ExtraInstanceBuf sphere = extra_buf("sphere", shapes.empty_sphere);

 public:
  LightProbePasses(SelectionType selection_type,
                   const ShapeCache &shapes,
                   const GlobalsUboStorage &theme_colors,
                   bool in_front)
      : OverlayPasses("Lights", selection_type, shapes, theme_colors, in_front){};

  virtual void begin_sync(Resources &res, const State &state) final override
  {
    OverlayPasses::begin_sync(res, state);
    dots_grid.begin_sync(res, state);
    groundline.clear();
  }

  virtual void end_sync(Resources &res, const State &state) final override
  {
    OverlayPasses::end_sync(res, state);
    if (groundline.data_buf.is_empty()) {
      return;
    }
    PassSimple::Sub &ps = ps_.sub("GroundLine");
    ps.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_BLEND_ADD |
                 state.clipping_state);
    ps.shader_set(res.shaders.extra_groundline.get());
    /* TODO: Fixed index. */
    ps.bind_ubo("globalsBlock", &res.globals_buf);
    groundline.end_sync(ps);
  }

  virtual void draw(Manager &manager, View &view, Framebuffer &fb) final override
  {
    OverlayPasses::draw(manager, view, fb);
    dots_grid.draw(manager, view, fb);
  }

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) final override
  {
    Object *ob = ob_ref.object;
    BLI_assert(ob->type == OB_LIGHTPROBE);

    ExtraInstanceData data(ob, res.object_wire_color(ob_ref, state));

    const ::LightProbe *probe = (::LightProbe *)ob_ref.object->data;
    const bool show_clipping = (probe->flag & LIGHTPROBE_FLAG_SHOW_CLIP_DIST) != 0;
    const bool show_parallax = (probe->flag & LIGHTPROBE_FLAG_SHOW_PARALLAX) != 0;
    const bool show_influence = (probe->flag & LIGHTPROBE_FLAG_SHOW_INFLUENCE) != 0;
    const bool show_data = (ob_ref.object->base_flag & BASE_SELECTED) ||
                           res.selection_type != SelectionType::DISABLED;

    if (probe->type == LIGHTPROBE_TYPE_CUBE) {
      /* Pack render data into object matrix. */
      data.matrix[2][3] = show_clipping ? probe->clipsta : -1.0;
      data.matrix[3][3] = show_clipping ? probe->clipend : -1.0;

      probe_cube.append(data, select_id);
      groundline.append(float4(data.matrix.location()), select_id);

      if (show_influence) {
        float influence_start = probe->distinf * (1.0f - probe->falloff);
        float influence_end = probe->distinf;

        ExtraInstanceBuf &buf = (probe->attenuation_type == LIGHTPROBE_SHAPE_BOX) ? cube : sphere;

        buf.append(data.with_size(influence_start), select_id);
        buf.append(data.with_size(influence_end), select_id);
      }

      if (show_parallax) {
        float radius = (probe->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) ? probe->distpar :
                                                                         probe->distinf;
        ExtraInstanceBuf &buf = (probe->parallax_type == LIGHTPROBE_SHAPE_BOX) ? cube : sphere;
        buf.append(data.with_size(radius), select_id);
      }
    }
    else if (probe->type == LIGHTPROBE_TYPE_GRID) {
      /* Pack render data into object matrix. */
      data.matrix[2][3] = show_clipping ? probe->clipsta : -1.0;
      data.matrix[3][3] = show_clipping ? probe->clipend : -1.0;
      probe_grid.append(data, select_id);

      if (show_influence) {
        float influence_start = 1.0f + probe->distinf * (1.0f - probe->falloff);
        float influence_end = 1.0f + probe->distinf;

        cube.append(data.with_size(influence_start), select_id);
        cube.append(data.with_size(influence_end), select_id);
      }

      /* Data dots */
      if (show_data) {
        int3 resolution = int3(&probe->grid_resolution_x);
        int theme_id = res.object_wire_theme_id(ob_ref, state) == TH_ACTIVE ? 1 : 2;
        dots_grid.object_sync(ob_ref, select_id, resolution, theme_id, res, state);
      }
    }
    else if (probe->type == LIGHTPROBE_TYPE_PLANAR) {
      probe_planar.append(data, select_id);

      if (res.selection_type != SelectionType::DISABLED &&
          (probe->flag & LIGHTPROBE_FLAG_SHOW_DATA)) {
        quad.append(data, select_id);
      }

      if (show_influence) {
        data.matrix.z_axis() = math::normalize(data.matrix.z_axis()) * probe->distinf;
        cube.append(data, select_id);
        data.matrix.z_axis() *= 1.0f - probe->falloff;
        cube.append(data, select_id);
      }

      data.matrix.z_axis() = float3(0);
      cube.append(data, select_id);

      data.matrix = float4x4(ob_ref.object->object_to_world);
      data.matrix.view<3, 3>() = math::normalize(data.matrix.view<3, 3>());
      single_arrow.append(data.with_size(ob_ref.object->empty_drawsize), select_id);
    }
  }
};

using LightProbe = OverlayType<LightProbePasses>;

}  // namespace blender::draw::overlay
