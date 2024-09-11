/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_paint.hh"
#include "DEG_depsgraph_query.hh"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Paints {

 private:
  /* Draw selection state on top of the mesh to communicate which areas can be painted on. */
  PassSimple paint_region_ps_ = {"paint_region_ps_"};
  PassSimple::Sub *paint_region_edge_ps_ = nullptr;
  PassSimple::Sub *paint_region_face_ps_ = nullptr;
  PassSimple::Sub *paint_region_vert_ps_ = nullptr;

  bool show_weight_ = false;
  bool show_wires_ = false;

  bool enabled_ = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    enabled_ =
        (state.space_type == SPACE_VIEW3D) && (res.selection_type == SelectionType::DISABLED) &&
        ELEM(state.ctx_mode, CTX_MODE_PAINT_WEIGHT, CTX_MODE_PAINT_VERTEX, CTX_MODE_PAINT_TEXTURE);

    if (!enabled_) {
      /* Not used. But release the data. */
      paint_region_ps_.init();
      return;
    }

    show_weight_ = state.ctx_mode == CTX_MODE_PAINT_WEIGHT;
    show_wires_ = state.overlay.paint_flag & V3D_OVERLAY_PAINT_WIRE;

    {
      auto &pass = paint_region_ps_;
      pass.init();
      {
        auto &sub = pass.sub("Face");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_face.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("ucolor", float4(1.0, 1.0, 1.0, 0.2));
        paint_region_face_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Edge");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_edge.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        paint_region_edge_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Vert");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_vert.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        paint_region_vert_ps_ = &sub;
      }
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State & /*state*/)
  {
    if (!enabled_) {
      return;
    }

    if (ob_ref.object->type != OB_MESH) {
      /* Only meshes are supported for now. */
      return;
    }

    ResourceHandle handle = {0};

    switch (ob_ref.object->mode) {
      case OB_MODE_WEIGHT_PAINT:
        handle = manager.resource_handle(ob_ref);
        break;
      case OB_MODE_VERTEX_PAINT:
        handle = manager.resource_handle(ob_ref);
        break;
      case OB_MODE_TEXTURE_PAINT:
        handle = manager.resource_handle(ob_ref);
        break;
      default:
        /* Not in paint mode. */
        return;
    }

    /* Selection Display. */
    {
      /* NOTE(fclem): Why do we need original mesh here, only to get the flag? */
      const Mesh &mesh_orig = *static_cast<Mesh *>(DEG_get_original_object(ob_ref.object)->data);
      const bool use_face_selection = (mesh_orig.editflag & ME_EDIT_PAINT_FACE_SEL);
      const bool use_vert_selection = (mesh_orig.editflag & ME_EDIT_PAINT_VERT_SEL);

      if (use_face_selection || show_wires_) {
        gpu::Batch *geom = DRW_cache_mesh_surface_edges_get(ob_ref.object);
        paint_region_edge_ps_->push_constant("useSelect", use_face_selection);
        paint_region_edge_ps_->draw(geom, handle);
      }
      if (use_face_selection) {
        gpu::Batch *geom = DRW_cache_mesh_surface_get(ob_ref.object);
        paint_region_face_ps_->draw(geom, handle);
      }
      if (use_vert_selection) {
        gpu::Batch *geom = DRW_cache_mesh_all_verts_get(ob_ref.object);
        paint_region_vert_ps_->draw(geom, handle);
      }
    }
  }

  void draw(GPUFrameBuffer *framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }
    GPU_framebuffer_bind(framebuffer);
    manager.submit(paint_region_ps_, view);
  }

 private:
  static bool paint_object_is_rendered_transparent(View3D *v3d, Object *ob)
  {
    if (v3d->shading.type == OB_WIRE) {
      return true;
    }
    if (v3d->shading.type == OB_SOLID) {
      if (v3d->shading.flag & V3D_SHADING_XRAY) {
        return true;
      }

      if (ob && v3d->shading.color_type == V3D_SHADING_OBJECT_COLOR) {
        return ob->color[3] < 1.0f;
      }

      /* NOTE: The active object might be hidden and hence have inconsistent evaluated state of its
       * mesh data. So only perform checks dependent on mesh after checking the object is actually
       * visible. */
      if (ob && ob->type == OB_MESH && BKE_object_is_visible_in_viewport(v3d, ob) && ob->data &&
          v3d->shading.color_type == V3D_SHADING_MATERIAL_COLOR)
      {
        Mesh *mesh = static_cast<Mesh *>(ob->data);
        for (int i = 0; i < mesh->totcol; i++) {
          Material *mat = BKE_object_material_get_eval(ob, i + 1);
          if (mat && mat->a < 1.0f) {
            return true;
          }
        }
      }
    }

    /* Check object display types. */
    if (ob && ELEM(ob->dt, OB_WIRE, OB_BOUNDBOX)) {
      return true;
    }

    return false;
  }
};

}  // namespace blender::draw::overlay
