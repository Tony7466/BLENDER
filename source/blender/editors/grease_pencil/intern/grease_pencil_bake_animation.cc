/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_anim_data.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_duplilist.hh"
#include "BKE_material.h"
#include "BKE_scene.hh"

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_set.hh"
#include "BLI_string.h"

#include "BLT_translation.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_anim_types.h"

#include "ED_grease_pencil.hh"
#include "ED_object.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_types.hh"

namespace blender::ed::greasepencil {
static void ensure_valid_frame_end(Main * /*main*/, Scene * /*scene*/, PointerRNA *ptr)
{
  const int frame_start = RNA_int_get(ptr, "frame_start");
  const int frame_end = RNA_int_get(ptr, "frame_end");

  if (frame_end <= frame_start) {
    RNA_int_set(ptr, "frame_end", frame_start + 1);
  }
}

static int bake_grease_pencil_animation_invoke(bContext *C,
                                               wmOperator *op,
                                               const wmEvent * /*event*/)
{
  PropertyRNA *prop;
  const Scene *scene = CTX_data_scene(C);

  prop = RNA_struct_find_property(op->ptr, "frame_start");
  if (!RNA_property_is_set(op->ptr, prop)) {
    const int frame_start = RNA_property_int_get(op->ptr, prop);
    if (frame_start < scene->r.sfra) {
      RNA_property_int_set(op->ptr, prop, scene->r.sfra);
    }
  }

  prop = RNA_struct_find_property(op->ptr, "frame_end");
  if (!RNA_property_is_set(op->ptr, prop)) {
    const int frame_end = RNA_property_int_get(op->ptr, prop);
    if (frame_end > scene->r.efra) {
      RNA_property_int_set(op->ptr, prop, scene->r.efra);
    }
  }

  return WM_operator_props_dialog_popup(
      C, op, 250, IFACE_("Bake Object Transform to Grease Pencil"), IFACE_("Bake"));
}

static Span<Object *> get_bake_targets(bContext &C,
                                       Depsgraph &depsgraph,
                                       Scene &scene,
                                       Vector<Object *> &bake_targets)
{
  bake_targets.clear();
  Object *active_object = CTX_data_active_object(&C);

  if (active_object->type == OB_GREASE_PENCIL) {
    bake_targets.append(active_object);
  }
  else if (active_object->type == OB_EMPTY) {
    ListBase *lb = object_duplilist(&depsgraph, &scene, active_object);
    LISTBASE_FOREACH (DupliObject *, duplicate_object, lb) {
      if (duplicate_object->ob->type != OB_GREASE_PENCIL) {
        continue;
      }

      bake_targets.append(duplicate_object->ob);
    }
    free_object_duplilist(lb);
  }

  CTX_DATA_BEGIN (&C, Object *, object, selected_objects) {
    if (object == active_object) {
      continue;
    }

    if (object->type == OB_GREASE_PENCIL) {
      bake_targets.append(object);
    }
    else if (object->type == OB_EMPTY) {
      ListBase *lb = object_duplilist(&depsgraph, &scene, active_object);
      LISTBASE_FOREACH (DupliObject *, duplicate_object, lb) {
        if (duplicate_object->ob->type != OB_GREASE_PENCIL) {
          continue;
        }

        bake_targets.append(duplicate_object->ob);
      }
      free_object_duplilist(lb);
    }
  }
  CTX_DATA_END;
  return bake_targets.as_span();
}

static void get_selected_object_keyframes(Span<Object *> bake_targets, Set<int> &r_keyframes)
{
  for (Object *bake_target : bake_targets) {
    AnimData *adt = BKE_animdata_from_id(&bake_target->id);
    if (adt == nullptr || adt->action == nullptr) {
      continue;
    }
    LISTBASE_FOREACH (FCurve *, fcurve, &adt->action->curves) {
      int i;
      BezTriple *bezt;
      for (i = 0, bezt = fcurve->bezt; i < fcurve->totvert; i++, bezt++) {
        if (bezt->f2 & SELECT) {
          r_keyframes.add(int(bezt->vec[1][0]));
        }
      }
    }
  }
}

static int bake_grease_pencil_animation_exec(bContext *C, wmOperator *op)
{
  using namespace bke::greasepencil;

  Main &bmain = *CTX_data_main(C);
  Depsgraph &depsgraph = *CTX_data_ensure_evaluated_depsgraph(C);
  Scene &scene = *CTX_data_scene(C);

  const int step = RNA_int_get(op->ptr, "step");

  const int frame_start = (scene.r.sfra > RNA_int_get(op->ptr, "frame_start")) ?
                              scene.r.sfra :
                              RNA_int_get(op->ptr, "frame_start");

  const int frame_end = (scene.r.efra < RNA_int_get(op->ptr, "frame_end")) ?
                            scene.r.efra :
                            RNA_int_get(op->ptr, "frame_end");

  const bool only_selected = RNA_boolean_get(op->ptr, "only_selected");
  const int frame_offset = RNA_int_get(op->ptr, "frame_target") - frame_start;
  const ReprojectMode reproject_mode = ReprojectMode(RNA_enum_get(op->ptr, "project_type"));

  View3D *v3d = CTX_wm_view3d(C);

  Vector<Object *> bake_targets;
  get_bake_targets(*C, depsgraph, scene, bake_targets);

  ushort local_view_bits = (v3d && v3d->localvd) ? v3d->local_view_uid : 0;
  Object *target_object = object::add_type(
      C, OB_GREASE_PENCIL, nullptr, scene.cursor.location, float3(0), false, local_view_bits);

  float4x4 invmat = math::invert(target_object->object_to_world());

  WM_cursor_wait(true);

  GreasePencil &target = *static_cast<GreasePencil *>(target_object->data);

  Set<int> keyframes;
  if (only_selected) {
    get_selected_object_keyframes(bake_targets, keyframes);
  }

  const int prior_frame = int(DEG_get_ctime(&depsgraph));
  int key = -1;

  for (int frame = frame_start; frame <= frame_end; frame++) {
    key++;
    if (key % step != 0 && frame != frame_end) {
      continue;
    }

    if (only_selected && !keyframes.contains(frame)) {
      continue;
    }

    scene.r.cfra = frame;
    BKE_scene_graph_update_for_newframe(&depsgraph);

    for (Object *source_object : bake_targets) {
      Object *ob_eval = DEG_get_evaluated_object(&depsgraph, source_object);
      GreasePencil &source_eval_grease_pencil = *static_cast<GreasePencil *>(ob_eval->data);

      for (const Layer *source_layer : source_eval_grease_pencil.layers()) {
        char *layer_name;
        BLI_SCOPED_DEFER([&] { MEM_SAFE_FREE(layer_name); });
        layer_name = BLI_sprintfN(
            "%s_%s", source_object->id.name + 2, source_layer->name().c_str());
        TreeNode *node = target.find_node_by_name(layer_name);
        if (node == nullptr) {
          target.add_layer(layer_name);
          target.add_empty_drawings(1);
        }

        Layer &target_layer = target.find_node_by_name(layer_name)->as_layer();

        const GreasePencilFrame *source_frame = source_layer->frame_at(scene.r.cfra);
        if (source_frame == nullptr) {
          continue;
        }

        const int target_frame_num = scene.r.cfra + frame_offset;
        GreasePencilFrame *target_frame = target_layer.add_frame(target_frame_num);
        Drawing &target_drawing = *target.get_editable_drawing_at(target_layer, target_frame_num);

        Drawing &source_drawing = *source_eval_grease_pencil.get_drawing_at(*source_layer,
                                                                            scene.r.cfra);
        const int duplicated_drawing_index = target.drawings().size();
        target.add_duplicate_drawings(1, source_drawing);

        GreasePencilFrame frame_duplicate = *source_frame;
        frame_duplicate.drawing_index = duplicated_drawing_index;

        target_frame->drawing_index = duplicated_drawing_index;
        target_frame->flag = source_frame->flag;
        target_frame->flag ^= GP_FRAME_SELECTED;
        target_frame->type = source_frame->type;

        bke::AttributeAccessor source_attributes = source_drawing.strokes().attributes();
        const VArray<int> source_material_indices = *source_attributes.lookup_or_default<int>(
            "material_index", bke::AttrDomain::Curve, 0);
        bke::CurvesGeometry &target_strokes = target_drawing.strokes_for_write();
        bke::SpanAttributeWriter<int> target_material_indices =
            target_strokes.attributes_for_write().lookup_or_add_for_write_span<int>(
                "material_index", bke::AttrDomain::Curve);

        for (const int i : target_drawing.strokes().curves_range()) {
          Material *source_material = BKE_object_material_get(source_object,
                                                              source_material_indices[i] + 1);
          BLI_assert(source_material != nullptr);

          bool found = false;
          for (const int target_index : IndexRange(target_object->totcol)) {
            Material *target_material = BKE_object_material_get(target_object, target_index + 1);
            if (source_material == target_material) {
              found = true;
              break;
            }
          }

          if (!found) {
            BKE_object_material_slot_add(&bmain, target_object);
            BKE_object_material_assign(&bmain,
                                       target_object,
                                       source_material,
                                       target_object->totcol,
                                       BKE_MAT_ASSIGN_USERPREF);
          }

          target_material_indices.span[i] = BKE_object_material_index_get(target_object,
                                                                          source_material);
        }

        target_material_indices.finish();
        for (float3 &pos : target_strokes.positions_for_write()) {
          pos = math::transform_point(ob_eval->object_to_world(), pos);
          pos = math::transform_point(invmat, pos);
        }
      }
    }
  }
  scene.r.cfra = prior_frame;
  BKE_scene_graph_update_for_newframe(&depsgraph);

  DEG_relations_tag_update(&bmain);
  DEG_id_tag_update(&scene.id, ID_RECALC_SELECT);
  DEG_id_tag_update(&target.id, ID_RECALC_SYNC_TO_EVAL);
  WM_event_add_notifier(C, NC_OBJECT | NA_ADDED, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, &scene);

  WM_cursor_wait(false);
  return OPERATOR_FINISHED;
}
static bool bake_grease_pencil_animation_poll(bContext *C)
{
  const Object *obact = CTX_data_active_object(C);
  if (CTX_data_mode_enum(C) != CTX_MODE_OBJECT) {
    return false;
  }

  /* Check if grease pencil or empty for dupli groups. */
  if ((obact == nullptr) || !ELEM(obact->type, OB_GREASE_PENCIL, OB_EMPTY)) {
    return false;
  }

  /* Only if the current view is 3D View. */
  const ScrArea *area = CTX_wm_area(C);
  return (area && area->spacetype);
}

static void GREASE_PENCIL_OT_bake_grease_pencil_animation(wmOperatorType *ot)
{
  ot->name = "Bake Object Transform to Grease Pencil";
  ot->idname = "GREASE_PENCIL_OT_bake_grease_pencil_animation";
  ot->description = "Bake grease pencil object transform to grease pencil keyframes";

  ot->invoke = bake_grease_pencil_animation_invoke;
  ot->exec = bake_grease_pencil_animation_exec;
  ot->poll = bake_grease_pencil_animation_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "frame_start", 1, 1, 100000, "Start Frame", "The start frame", 1, 100000);

  PropertyRNA *prop = RNA_def_int(
      ot->srna, "frame_end", 250, 1, 100000, "End Frame", "The end frame of animation", 1, 100000);
  RNA_def_property_update_runtime(prop, ensure_valid_frame_end);

  RNA_def_int(ot->srna, "step", 1, 1, 100, "Step", "Step between generated frames", 1, 100);

  RNA_def_boolean(ot->srna,
                  "only_selected",
                  false,
                  "Only Selected Keyframes",
                  "Convert only selected keyframes");
  RNA_def_int(
      ot->srna, "frame_target", 1, 1, 100000, "Target Frame", "Destination frame", 1, 100000);

  static const EnumPropertyItem rna_grease_pencil_reproject_type_items[] = {
      {int(ReprojectMode::Keep), "KEEP", 0, "No Reproject", ""},
      {int(ReprojectMode::Front),
       "FRONT",
       0,
       "Front",
       "Reproject the strokes using the X-Z plane"},
      {int(ReprojectMode::Side), "SIDE", 0, "Side", "Reproject the strokes using the Y-Z plane"},
      {int(ReprojectMode::Top), "TOP", 0, "Top", "Reproject the strokes using the X-Y plane"},
      {int(ReprojectMode::View),
       "VIEW",
       0,
       "View",
       "Reproject the strokes to end up on the same plane, as if drawn from the current "
       "viewpoint "
       "using 'Cursor' Stroke Placement"},
      {int(ReprojectMode::Cursor),
       "CURSOR",
       0,
       "Cursor",
       "Reproject the strokes using the orientation of 3D cursor"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_enum(ot->srna,
               "project_type",
               rna_grease_pencil_reproject_type_items,
               int(ReprojectMode::Keep),
               "Projection Type",
               "");
}
}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_bake_animation()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_bake_grease_pencil_animation);
}
