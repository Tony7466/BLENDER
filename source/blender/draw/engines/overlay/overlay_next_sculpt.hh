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

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Sculpts {

 private:
  const SelectionType selection_type_;

  PassMain ps_ = {"Sculpt"};

  bool enabled_ = false;

 public:
  Sculpts(const SelectionType selection_type_) : selection_type_(selection_type_) {}

  void begin_sync(Resources & /*res*/, const State &state)
  {
    enabled_ = (state.space_type == SPACE_VIEW3D) &&
               (state.overlay.flag & V3D_OVERLAY_FACE_ORIENTATION) && !state.xray_enabled &&
               (selection_type_ == SelectionType::DISABLED);
    if (!enabled_) {
      /* Not used. But release the data. */
      ps_.init();
      return;
    }

    /* TODO init. */
  }

  void object_sync(const ObjectRef &ob_ref, const State &state)
  {
    if (!enabled_) {
      return;
    }

    if (ob_ref.object->type != OB_MESH) {
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

    const SculptSession &sculpt_session = *ob_ref.object->sculpt;
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
        const SubdivCCG &subdiv_ccg = *sculpt_session.subdiv_ccg;
        const Mesh &base_mesh = *static_cast<const Mesh *>(object_orig->data);
        if (!BKE_subdiv_ccg_key_top_level(subdiv_ccg).has_mask &&
            !base_mesh.attributes().contains(".sculpt_face_set"))
        {
          return;
        }
        break;
      }
      case blender::bke::pbvh::Type::BMesh: {
        const BMesh &bm = *sculpt_session.bm;
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
      /* TODO Draw pbvh. */
    }
    else {
      Mesh &mesh = *static_cast<Mesh *>(ob_ref.object->data);
      gpu::Batch *sculpt_overlays = DRW_mesh_batch_cache_get_sculpt_overlays(mesh);
      /* TODO Draw pbvh. */
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }
    GPU_framebuffer_bind(framebuffer);
    manager.submit(ps_, view);
  }
};

}  // namespace blender::draw::overlay
