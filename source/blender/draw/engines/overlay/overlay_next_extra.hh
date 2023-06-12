/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_extra_pass.hh"

#include "overlay_next_bounds.hh"
#include "overlay_next_camera.hh"
#include "overlay_next_collision.hh"
#include "overlay_next_empty.hh"
#include "overlay_next_force_field.hh"
#include "overlay_next_light.hh"
#include "overlay_next_light_probe.hh"
#include "overlay_next_object_center.hh"
#include "overlay_next_object_relation.hh"

namespace blender::draw::overlay {

class Extra {
 public:
  ExtraInstancePass pass_;
  ExtraInstancePass pass_in_front_;

  Extra(const SelectionType selection_type,
        const ShapeCache &shapes,
        const GlobalsUboStorage &theme_colors)
      : pass_(selection_type, shapes, theme_colors, "Extra Shapes"),
        pass_in_front_(selection_type, shapes, theme_colors, "Extra Shapes In Front"){};

  void begin_sync()
  {
    pass_.begin_sync();
    pass_in_front_.begin_sync();
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources &res,
                   const State &state)
  {
    ExtraInstancePass &pass = ob_ref.object->dtx & OB_DRAW_IN_FRONT ? pass_in_front_ : pass_;

    ExtraInstanceData data(ob_ref.object, res.object_wire_color(ob_ref, state));

    switch (ob_ref.object->type) {
      case OB_CAMERA:
        camera_sync(ob_ref, select_id, res, state, pass, data);
        break;
      case OB_EMPTY:
        empty_sync(ob_ref, select_id, res, state, pass, data);
        break;
      case OB_LAMP:
        light_sync(ob_ref, select_id, res, state, pass, data);
        break;
      case OB_LIGHTPROBE:
        probe_sync(ob_ref, select_id, res, state, pass, data);
        break;
      case OB_SPEAKER:
        pass.speaker.append(data, select_id);
        break;
    }

    const Scene *scene = state.scene;
    Object *ob = ob_ref.object;
    ModifierData *md = nullptr;

    const bool is_select_mode = state.selection_type != SelectionType::DISABLED;
    const bool is_paint_mode = (state.object_mode &
                                (OB_MODE_ALL_PAINT | OB_MODE_ALL_PAINT_GPENCIL |
                                 OB_MODE_SCULPT_CURVES)) != 0;
    const bool from_dupli = (ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI)) != 0;
    const bool has_bounds = !ELEM(
        ob->type, OB_LAMP, OB_CAMERA, OB_EMPTY, OB_SPEAKER, OB_LIGHTPROBE);
    const bool has_texspace = has_bounds &&
                              !ELEM(
                                  ob->type, OB_EMPTY, OB_LATTICE, OB_ARMATURE, OB_GPENCIL_LEGACY);
    const bool draw_relations = ((state.v3d_flag & V3D_HIDE_HELPLINES) == 0) && !is_select_mode;
    const bool draw_obcenters = !is_paint_mode &&
                                (state.overlay.flag & V3D_OVERLAY_HIDE_OBJECT_ORIGINS) == 0;
    const bool draw_texspace = (ob->dtx & OB_TEXSPACE) && has_texspace;
    const bool draw_obname = (ob->dtx & OB_DRAWNAME) && DRW_state_show_text();
    const bool draw_bounds = has_bounds && ((ob->dt == OB_BOUNDBOX) ||
                                            ((ob->dtx & OB_DRAWBOUNDOX) && !from_dupli));
    const bool draw_xform = state.object_mode == OB_MODE_OBJECT &&
                            (scene->toolsettings->transform_flag & SCE_XFORM_DATA_ORIGIN) &&
                            (ob->base_flag & BASE_SELECTED) && !is_select_mode;
#if 0
    /* TODO */
    /* Don't show fluid domain overlay extras outside of cache range. */
    const bool draw_volume =
        !from_dupli && (md = BKE_modifiers_findby_type(ob, eModifierType_Fluid)) &&
        BKE_modifier_is_enabled(scene, md, eModifierMode_Realtime) &&
        (((FluidModifierData *)md)->domain != nullptr) &&
        (scene->r.cfra >= (((FluidModifierData *)md)->domain->cache_frame_start)) &&
        (scene->r.cfra <= (((FluidModifierData *)md)->domain->cache_frame_end));
#endif

    if (draw_bounds) {
      bounds_sync(ob_ref, select_id, res, state, pass, data);
    }

    /* Helpers for when we're transforming origins. */
    if (draw_xform) {
      /* TODO(Miguel Pozo): What's this? */
      const float4 color_xform = {0.15f, 0.15f, 0.15f, 0.7f};
      pass.origin_xform.append(data.with_color(color_xform), select_id);
    }
    if (ob->pd && ob->pd->forcefield) {
      force_field_sync(ob_ref, select_id, res, state, pass, data);
    }

    /* don't show object extras in set's */
    if (!from_dupli) {
      if (draw_obcenters) {
        object_center_sync(ob_ref, select_id, res, state, pass);
      }
      if (draw_relations) {
        object_relation_sync(ob_ref, select_id, res, state, pass);
      }
      if (draw_obname) {
        /* TODO
        OVERLAY_object_name(ob, theme_id);
        */
      }
      if (draw_texspace) {
        /* TODO
        OVERLAY_texture_space(cb, ob, color);
        */
      }
      if (ob->rigidbody_object != nullptr) {
        collision_sync(ob_ref, select_id, res, state, pass, data);
      }
      if (ob->dtx & OB_AXIS) {
        pass.arrows.append(data, select_id);
      }
      /* TODO
      if (draw_volume) {
        OVERLAY_volume_extra(cb, vedata, ob, md, scene, color);
      }
      */
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    pass_.end_sync(res, state);
    pass_in_front_.end_sync(res, state);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    pass_.draw(manager, view, res.overlay_line_fb);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    pass_in_front_.draw(manager, view, res.overlay_line_in_front_fb);
  }
};

}  // namespace blender::draw::overlay
