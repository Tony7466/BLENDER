/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"
#include "BKE_subdiv_ccg.hh"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Sculpts {

 private:
  const SelectionType selection_type_;

  PassSimple pass_ = {"Sculpt"};

  SculptBatchFeature sculpt_batch_features_ = SCULPT_BATCH_DEFAULT;

  bool enabled_ = false;

 public:
  Sculpts(const SelectionType selection_type_) : selection_type_(selection_type_) {}

  void begin_sync(Resources &res, const State &state)
  {
    const int sculpt_overlay_flags = V3D_OVERLAY_SCULPT_SHOW_FACE_SETS |
                                     V3D_OVERLAY_SCULPT_SHOW_MASK;

    enabled_ = (state.space_type == SPACE_VIEW3D) && !state.xray_enabled &&
               (selection_type_ == SelectionType::DISABLED) &&
               ELEM(state.object_mode, OB_MODE_SCULPT_CURVES, OB_MODE_SCULPT) &&
               (state.overlay.flag & sculpt_overlay_flags);

    if (!enabled_) {
      /* Not used. But release the data. */
      pass_.init();
      return;
    }

    const bool show_face_set = state.overlay.flag & V3D_OVERLAY_SCULPT_SHOW_FACE_SETS;
    const bool show_mask = state.overlay.flag & V3D_OVERLAY_SCULPT_SHOW_MASK;
    float face_set_opacity = show_face_set ? state.overlay.sculpt_mode_face_sets_opacity : 0.0f;
    float mask_opacity = show_mask ? state.overlay.sculpt_mode_mask_opacity : 0.0f;

    sculpt_batch_features_ = (show_face_set ? SCULPT_BATCH_FACE_SET : SCULPT_BATCH_DEFAULT) |
                             (show_mask ? SCULPT_BATCH_MASK : SCULPT_BATCH_DEFAULT);

    pass_.init();
    pass_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_MUL,
                    state.clipping_plane_count);
    {
      auto &pass = pass_;
      pass.shader_set(res.shaders.sculpt_mesh.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      pass.push_constant("maskOpacity", mask_opacity);
      pass.push_constant("faceSetsOpacity", face_set_opacity);
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State &state)
  {
    if (!enabled_) {
      return;
    }

    if (ob_ref.object->type != OB_MESH) {
      /* Only meshes are supported by this overlay. */
      return;
    }

    const SculptSession *sculpt_session = ob_ref.object->sculpt;
    if (sculpt_session == nullptr) {
      return;
    }

    bke::pbvh::Tree *pbvh = bke::object::pbvh_get(*ob_ref.object);
    if (!pbvh) {
      /* It is possible to have SculptSession without pbvh::Tree. This happens, for example, when
       * toggling object mode to sculpt then to edit mode. */
      return;
    }

    /* Using the original object/geometry is necessary because we skip depsgraph updates in sculpt
     * mode to improve performance. This means the evaluated mesh doesn't have the latest face set,
     * visibility, and mask data. */
    Object *object_orig = reinterpret_cast<Object *>(DEG_get_original_id(&ob_ref.object->id));
    if (!object_orig) {
      BLI_assert_unreachable();
      return;
    }

    switch (pbvh->type()) {
      case blender::bke::pbvh::Type::Mesh: {
        const Mesh &mesh = *static_cast<const Mesh *>(object_orig->data);
        if (!mesh.attributes().contains(".sculpt_face_set") &&
            !mesh.attributes().contains(".sculpt_mask"))
        {
          return;
        }
        break;
      }
      case blender::bke::pbvh::Type::Grids: {
        const SubdivCCG &subdiv_ccg = *sculpt_session->subdiv_ccg;
        const Mesh &base_mesh = *static_cast<const Mesh *>(object_orig->data);
        if (!BKE_subdiv_ccg_key_top_level(subdiv_ccg).has_mask &&
            !base_mesh.attributes().contains(".sculpt_face_set"))
        {
          return;
        }
        break;
      }
      case blender::bke::pbvh::Type::BMesh: {
        const BMesh &bm = *sculpt_session->bm;
        if (!CustomData_has_layer_named(&bm.pdata, CD_PROP_FLOAT, ".sculpt_face_set") &&
            !CustomData_has_layer_named(&bm.vdata, CD_PROP_FLOAT, ".sculpt_mask"))
        {
          return;
        }
        break;
      }
    }

    const bool use_pbvh = BKE_sculptsession_use_pbvh_draw(ob_ref.object, state.rv3d);
    if (use_pbvh) {
      /* TODO(fclem): Deduplicate with other engine. */
      const blender::Bounds<float3> bounds = bke::pbvh::bounds_get(*ob_ref.object->sculpt->pbvh);
      const float3 center = math::midpoint(bounds.min, bounds.max);
      const float3 half_extent = bounds.max - center;
      ResourceHandle handle = manager.resource_handle(ob_ref, nullptr, &center, &half_extent);

      for (SculptBatch &batch : sculpt_batches_get(ob_ref.object, sculpt_batch_features_)) {
        pass_.draw(batch.batch, handle);
      }
    }
    else {
      ResourceHandle handle = manager.resource_handle(ob_ref);

      Mesh &mesh = *static_cast<Mesh *>(ob_ref.object->data);
      gpu::Batch *sculpt_overlays = DRW_mesh_batch_cache_get_sculpt_overlays(mesh);
      pass_.draw(sculpt_overlays, handle);
    }
  }

  void draw_on_render(GPUFrameBuffer *framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }
    GPU_framebuffer_bind(framebuffer);
    manager.submit(pass_, view);
  }
};

}  // namespace blender::draw::overlay
