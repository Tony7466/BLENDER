/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_attribute.hh"
#include "BKE_curves.h"

#include "DNA_curves_types.h"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Curves {
 private:
  /* New curve type. */
  PassSimple edit_curves_ps_ = {"Curve Edit"};
  PassSimple::Sub *edit_curves_points_ps_ = nullptr;
  PassSimple::Sub *edit_curves_lines_ps_ = nullptr;
  PassSimple::Sub *edit_curves_handles_ps_ = nullptr;

  bool xray_enabled = false;

  /* TODO(fclem): This is quite wasteful and expensive, prefer in shader Z modification like the
   * retopology offset. */
  View view_edit_cage = {"view_edit_cage"};
  float view_dist = 0.0f;

 public:
  void begin_sync(Resources &res, const State &state, const View &view)
  {
    view_dist = state.view_dist_get(view.winmat());
    xray_enabled = state.xray_enabled;

    {
      auto &pass = edit_curves_ps_;
      pass.init();

      {
        auto &sub = pass.sub("Points");
        pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                       DRW_STATE_WRITE_DEPTH | state.clipping_state);
        sub.shader_set(res.shaders.curve_edit_points.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("useWeight", false);
        sub.push_constant("useGreasePencil", false);
        edit_curves_points_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Lines");
        pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                       DRW_STATE_WRITE_DEPTH | state.clipping_state);
        sub.shader_set(res.shaders.curve_edit_line.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("useWeight", false);
        sub.push_constant("useGreasePencil", false);
        edit_curves_lines_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Handles");
        pass.state_set(DRW_STATE_WRITE_COLOR | state.clipping_state);
        sub.shader_set(res.shaders.curve_edit_handles.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        edit_curves_handles_ps_ = &sub;
      }
    }
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref, Resources & /*res*/)
  {
    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    Object *ob = ob_ref.object;
    ::Curves &curves = *static_cast<::Curves *>(ob->data);
    const bool show_points = bke::AttrDomain(curves.selection_domain) == bke::AttrDomain::Point;

    GPUUniformBuf *ubo_storage = DRW_curves_batch_cache_ubo_storage(&curves);

    if (show_points) {
      gpu::Batch *geom = DRW_curves_batch_cache_get_edit_points(&curves);
      edit_curves_points_ps_->draw(geom, res_handle);
    }
    {
      gpu::Batch *geom = DRW_curves_batch_cache_get_edit_curves_handles(&curves);
      edit_curves_handles_ps_->bind_ubo("curvesInfoBlock", ubo_storage);
      edit_curves_handles_ps_->draw(geom, res_handle);
    }
    {
      gpu::Batch *geom = DRW_curves_batch_cache_get_edit_curves_lines(&curves);
      edit_curves_lines_ps_->bind_ubo("curvesInfoBlock", ubo_storage);
      edit_curves_lines_ps_->draw(geom, res_handle);
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    view_edit_cage.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 0.5f));

    GPU_framebuffer_bind(framebuffer);
    manager.submit(edit_curves_ps_, view_edit_cage);
  }
};

}  // namespace blender::draw::overlay
