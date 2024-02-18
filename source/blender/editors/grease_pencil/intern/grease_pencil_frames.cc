/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_utildefines.h"

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"

#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"
#include "DEG_depsgraph.hh"

#include "DNA_curves_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_scene_types.h"

#include "ED_grease_pencil.hh"
#include "ED_keyframes_edit.hh"
#include "ED_markers.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include <algorithm>
#include <optional>

namespace blender::ed::greasepencil {

void set_selected_frames_type(bke::greasepencil::Layer &layer,
                              const eBezTriple_KeyframeType key_type)
{
  for (GreasePencilFrame &frame : layer.frames_for_write().values()) {
    if (frame.is_selected()) {
      frame.type = key_type;
    }
  }
}

static float get_snapped_frame_number(const float frame_number,
                                      Scene &scene,
                                      const eEditKeyframes_Snap mode)
{
  switch (mode) {
    case SNAP_KEYS_CURFRAME: /* Snap to current frame. */
      return scene.r.cfra;
    case SNAP_KEYS_NEARSEC: /* Snap to nearest second. */
    {
      float secf = (scene.r.frs_sec / scene.r.frs_sec_base);
      return floorf(frame_number / secf + 0.5f) * secf;
    }
    case SNAP_KEYS_NEARMARKER: /* Snap to nearest marker. */
      return ED_markers_find_nearest_marker_time(&scene.markers, frame_number);
    default:
      break;
  }
  return frame_number;
}

bool snap_selected_frames(GreasePencil &grease_pencil,
                          bke::greasepencil::Layer &layer,
                          Scene &scene,
                          const eEditKeyframes_Snap mode)
{
  bool changed = false;
  blender::Map<int, int> frame_number_destinations;
  for (auto [frame_number, frame] : layer.frames().items()) {
    if (!frame.is_selected()) {
      continue;
    }
    const int snapped = round_fl_to_int(
        get_snapped_frame_number(float(frame_number), scene, mode));
    if (snapped != frame_number) {
      frame_number_destinations.add(frame_number, snapped);
      changed = true;
    }
  }

  if (changed) {
    grease_pencil.move_frames(layer, frame_number_destinations);
  }

  return changed;
}

static int get_mirrored_frame_number(const int frame_number,
                                     const Scene &scene,
                                     const eEditKeyframes_Mirror mode,
                                     const TimeMarker *first_selected_marker)
{
  switch (mode) {
    case MIRROR_KEYS_CURFRAME: /* Mirror over current frame. */
      return 2 * scene.r.cfra - frame_number;
    case MIRROR_KEYS_XAXIS:
    case MIRROR_KEYS_YAXIS: /* Mirror over frame 0. */
      return -frame_number;
    case MIRROR_KEYS_MARKER: /* Mirror over marker. */
      if (first_selected_marker == nullptr) {
        break;
      }
      return 2 * first_selected_marker->frame - frame_number;
    default:
      break;
  }
  return frame_number;
}

bool mirror_selected_frames(GreasePencil &grease_pencil,
                            bke::greasepencil::Layer &layer,
                            Scene &scene,
                            const eEditKeyframes_Mirror mode)
{
  bool changed = false;
  Map<int, int> frame_number_destinations;

  /* Pre-compute the first selected marker, so that we don't compute it for each frame. */
  const TimeMarker *first_selected_marker = (mode == MIRROR_KEYS_MARKER) ?
                                                ED_markers_get_first_selected(&scene.markers) :
                                                nullptr;

  for (auto [frame_number, frame] : layer.frames().items()) {
    if (!frame.is_selected()) {
      continue;
    }

    const int mirrored_frame_number = get_mirrored_frame_number(
        frame_number, scene, mode, first_selected_marker);

    if (mirrored_frame_number != frame_number) {
      frame_number_destinations.add(frame_number, mirrored_frame_number);
      changed = true;
    }
  }

  if (changed) {
    grease_pencil.move_frames(layer, frame_number_destinations);
  }

  return changed;
}

bool duplicate_selected_frames(GreasePencil &grease_pencil, bke::greasepencil::Layer &layer)
{
  using namespace bke::greasepencil;
  bool changed = false;
  LayerTransformData &trans_data = layer.runtime->trans_data_;

  for (auto [frame_number, frame] : layer.frames_for_write().items()) {
    if (!frame.is_selected()) {
      continue;
    }

    /* Create the duplicate drawing. */
    const Drawing *drawing = grease_pencil.get_editable_drawing_at(&layer, frame_number);
    if (drawing == nullptr) {
      continue;
    }
    const int duplicated_drawing_index = grease_pencil.drawings().size();
    grease_pencil.add_duplicate_drawings(1, *drawing);

    /* Make a copy of the frame in the duplicates. */
    GreasePencilFrame frame_duplicate = frame;
    frame_duplicate.drawing_index = duplicated_drawing_index;
    trans_data.temp_frames_buffer.add_overwrite(frame_number, frame_duplicate);

    /* Deselect the current frame, so that only the copy is selected. */
    frame.flag ^= GP_FRAME_SELECTED;

    changed = true;
  }

  return changed;
}

bool remove_all_selected_frames(GreasePencil &grease_pencil, bke::greasepencil::Layer &layer)
{
  Vector<int> frames_to_remove;
  for (auto [frame_number, frame] : layer.frames().items()) {
    if (!frame.is_selected()) {
      continue;
    }
    frames_to_remove.append(frame_number);
  }
  return grease_pencil.remove_frames(layer, frames_to_remove.as_span());
}

static void select_frame(GreasePencilFrame &frame, const short select_mode)
{
  switch (select_mode) {
    case SELECT_ADD:
      frame.flag |= GP_FRAME_SELECTED;
      break;
    case SELECT_SUBTRACT:
      frame.flag &= ~GP_FRAME_SELECTED;
      break;
    case SELECT_INVERT:
      frame.flag ^= GP_FRAME_SELECTED;
      break;
  }
}

bool select_frame_at(bke::greasepencil::Layer &layer,
                     const int frame_number,
                     const short select_mode)
{

  GreasePencilFrame *frame = layer.frames_for_write().lookup_ptr(frame_number);
  if (frame == nullptr) {
    return false;
  }
  select_frame(*frame, select_mode);
  return true;
}

void select_frames_at(bke::greasepencil::LayerGroup &layer_group,
                      const int frame_number,
                      const short select_mode)
{
  LISTBASE_FOREACH_BACKWARD (GreasePencilLayerTreeNode *, node_, &layer_group.children) {
    bke::greasepencil::TreeNode &node = node_->wrap();
    if (node.is_group()) {
      select_frames_at(node.as_group(), frame_number, select_mode);
    }
    else if (node.is_layer()) {
      select_frame_at(node.as_layer(), frame_number, select_mode);
    }
  }
}

void select_all_frames(bke::greasepencil::Layer &layer, const short select_mode)
{
  for (auto item : layer.frames_for_write().items()) {
    select_frame(item.value, select_mode);
  }
}

bool has_any_frame_selected(const bke::greasepencil::Layer &layer)
{
  for (const auto &[frame_number, frame] : layer.frames().items()) {
    if (frame.is_selected()) {
      return true;
    }
  }
  return false;
}

void select_frames_region(KeyframeEditData *ked,
                          bke::greasepencil::TreeNode &node,
                          const short tool,
                          const short select_mode)
{
  if (node.is_layer()) {
    for (auto [frame_number, frame] : node.as_layer().frames_for_write().items()) {
      /* Construct a dummy point coordinate to do this testing with. */
      const float2 pt(float(frame_number), ked->channel_y);

      /* Check the necessary regions. */
      if (tool == BEZT_OK_CHANNEL_LASSO) {
        if (keyframe_region_lasso_test(static_cast<const KeyframeEdit_LassoData *>(ked->data), pt))
        {
          select_frame(frame, select_mode);
        }
      }
      else if (tool == BEZT_OK_CHANNEL_CIRCLE) {
        if (keyframe_region_circle_test(static_cast<const KeyframeEdit_CircleData *>(ked->data),
                                        pt)) {
          select_frame(frame, select_mode);
        }
      }
    }
  }
  else if (node.is_group()) {
    LISTBASE_FOREACH_BACKWARD (GreasePencilLayerTreeNode *, node_, &node.as_group().children) {
      select_frames_region(ked, node_->wrap(), tool, select_mode);
    }
  }
}

void select_frames_range(bke::greasepencil::TreeNode &node,
                         const float min,
                         const float max,
                         const short select_mode)
{
  /* Only select those frames which are in bounds. */
  if (node.is_layer()) {
    for (auto [frame_number, frame] : node.as_layer().frames_for_write().items()) {
      if (IN_RANGE(float(frame_number), min, max)) {
        select_frame(frame, select_mode);
      }
    }
  }
  else if (node.is_group()) {
    LISTBASE_FOREACH_BACKWARD (GreasePencilLayerTreeNode *, node_, &node.as_group().children) {
      select_frames_range(node_->wrap(), min, max, select_mode);
    }
  }
}

static void append_frame_to_key_edit_data(KeyframeEditData *ked,
                                          const int frame_number,
                                          const GreasePencilFrame &frame)
{
  CfraElem *ce = MEM_cnew<CfraElem>(__func__);
  ce->cfra = float(frame_number);
  ce->sel = frame.is_selected();
  BLI_addtail(&ked->list, ce);
}

void create_keyframe_edit_data_selected_frames_list(KeyframeEditData *ked,
                                                    const bke::greasepencil::Layer &layer)
{
  BLI_assert(ked != nullptr);

  for (const auto &[frame_number, frame] : layer.frames().items()) {
    if (frame.is_selected()) {
      append_frame_to_key_edit_data(ked, frame_number, frame);
    }
  }
}

static int insert_blank_frame_exec(bContext *C, wmOperator *op)
{
  using namespace blender::bke::greasepencil;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  const int current_frame = scene->r.cfra;
  const bool all_layers = RNA_boolean_get(op->ptr, "all_layers");
  const int duration = RNA_int_get(op->ptr, "duration");

  bool changed = false;
  if (all_layers) {
    for (Layer *layer : grease_pencil.layers_for_write()) {
      if (!layer->is_editable()) {
        continue;
      }
      changed = grease_pencil.insert_blank_frame(
          *layer, current_frame, duration, BEZT_KEYTYPE_KEYFRAME);
    }
  }
  else {
    if (!grease_pencil.has_active_layer()) {
      return OPERATOR_CANCELLED;
    }
    changed = grease_pencil.insert_blank_frame(
        *grease_pencil.get_active_layer(), current_frame, duration, BEZT_KEYTYPE_KEYFRAME);
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);
    WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
  }

  return OPERATOR_FINISHED;
}

static bool attributes_varrays_are_equal(const bke::GAttributeReader &attrs_a,
                                         const bke::GAttributeReader &attrs_b)
{
  if (attrs_a.varray.size() != attrs_b.varray.size() ||
      attrs_a.varray.type() != attrs_b.varray.type())
  {
    return false;
  }

  if (attrs_a.varray.is_span() && attrs_b.varray.is_span()) {
    if (attrs_a.varray.get_internal_span().size() != attrs_b.varray.get_internal_span().size()) {
      return false;
    }

    if (attrs_a.varray.get_internal_span().data() != attrs_b.varray.get_internal_span().data()) {
      return false;
    }
  }

  return true;
}

template<typename T>
static bool attributes_elements_are_equal(const bke::GAttributeReader &attrs_a,
                                          const bke::GAttributeReader &attrs_b)
{
  const VArray attributes_a = attrs_a.varray.typed<T>();
  const VArray attributes_b = attrs_b.varray.typed<T>();

  const std::optional<T> value_a = attributes_a.get_if_single();
  const std::optional<T> value_b = attributes_b.get_if_single();
  if (value_a.has_value() && value_b.has_value()) {
    return value_a.value() == value_b.value();
  }

  const VArraySpan attrs_span_a = attributes_a;
  const VArraySpan attrs_span_b = attributes_b;

  return std::equal(
      attrs_span_a.begin(), attrs_span_a.end(), attrs_span_b.begin(), attrs_span_b.end());
}

static bool curves_geometry_is_equal(const bke::CurvesGeometry &curves_a,
                                     const bke::CurvesGeometry &curves_b)
{
  using namespace blender::bke;

  if (curves_a.curves_num() != curves_b.curves_num() ||
      curves_a.points_num() != curves_b.points_num() || curves_a.offsets() != curves_b.offsets())
  {
    return false;
  }

  const AttributeAccessor attributes_a = curves_a.attributes();
  const AttributeAccessor attributes_b = curves_b.attributes();

  const Set<AttributeIDRef> ids_a = attributes_a.all_ids();
  const Set<AttributeIDRef> ids_b = attributes_b.all_ids();
  if (ids_a != ids_b) {
    return false;
  }

  for (const AttributeIDRef &id : ids_a) {
    GAttributeReader attrs_a = attributes_a.lookup(id);
    GAttributeReader attrs_b = attributes_b.lookup(id);

    if (!attributes_varrays_are_equal(attrs_a, attrs_b)) {
      return false;
    }
  }

  return true;

  // return attributes_a.for_all([&](const AttributeIDRef &id, const AttributeMetaData) {
  //   GAttributeReader attrs_a = attributes_a.lookup(id);
  //   GAttributeReader attrs_b = attributes_b.lookup(id);

  //   if (!attributes_varrays_are_equal(attrs_a, attrs_b)) {
  //     return false;
  //   }

  //   bool attrs_equal = true;

  //   attribute_math::convert_to_static_type(attrs_a.varray.type(), [&](auto dummy) {
  //     using T = decltype(dummy);

  //     attrs_equal = attributes_elements_are_equal<T>(attrs_a, attrs_b);
  //   });

  //   return attrs_equal;
  // });
}

static int frame_clean_duplicate_exec(bContext *C, wmOperator *op)
{
  using namespace blender::bke::greasepencil;
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  const bool selected = RNA_boolean_get(op->ptr, "selected");

  bool changed = false;

  for (Layer *layer : grease_pencil.layers_for_write()) {
    if (!layer->is_editable()) {
      continue;
    }

    const Span<FramesMapKey> &keys = layer->sorted_keys();
    Vector<FramesMapKey> frames_to_delete = {};

    for (size_t i = 0; i < keys.size(); ++i) {
      if (i + 1 >= keys.size()) {
        break;
      }

      FramesMapKey current = keys[i];
      FramesMapKey next = keys[i + 1];

      GreasePencilFrame frame = layer->frames().lookup(current);

      if (selected && !frame.is_selected()) {
        continue;
      }

      Drawing *drawing = grease_pencil.get_editable_drawing_at(layer, current);
      Drawing *drawing_next = grease_pencil.get_editable_drawing_at(layer, next);

      if (!drawing || !drawing_next) {
        continue;
      }

      bke::CurvesGeometry &curves = drawing->strokes_for_write();
      bke::CurvesGeometry &curves_next = drawing_next->strokes_for_write();

      if (!curves_geometry_is_equal(curves, curves_next)) {
        continue;
      }

      frames_to_delete.append(next);
    }

    for (const FramesMapKey frame : frames_to_delete) {
      layer->remove_frame(frame);
    }

    changed = true;
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);
    WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
  }

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_insert_blank_frame(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Insert Blank Frame";
  ot->idname = "GREASE_PENCIL_OT_insert_blank_frame";
  ot->description = "Insert a blank frame on the current scene frame";

  /* callbacks */
  ot->exec = insert_blank_frame_exec;
  ot->poll = active_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  prop = RNA_def_boolean(
      ot->srna, "all_layers", false, "All Layers", "Insert a blank frame in all editable layers");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  RNA_def_int(ot->srna, "duration", 0, 0, MAXFRAME, "Duration", "", 0, 100);
}

static void GREASE_PENCIL_OT_frame_clean_duplicate(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Delete Duplicate Frames";
  ot->idname = "GREASE_PENCIL_OT_frame_clean_duplicate";
  ot->description = "Remove any keyframe that is a duplicate of the previous one";

  /* callbacks */
  ot->exec = frame_clean_duplicate_exec;
  ot->poll = active_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  prop = RNA_def_boolean(
      ot->srna, "selected", false, "Selected", "Only delete selected keyframes");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_frames()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_insert_blank_frame);
  WM_operatortype_append(GREASE_PENCIL_OT_frame_clean_duplicate);
}
