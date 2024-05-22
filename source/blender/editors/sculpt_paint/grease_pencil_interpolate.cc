/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_report.hh"

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"
#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"

#include "DNA_grease_pencil_types.h"

#include "ED_grease_pencil.hh"
#include "ED_numinput.hh"
#include "ED_screen.hh"

#include "GEO_interpolate_curves.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include <climits>
#include <iostream>

namespace blender::ed::sculpt_paint::greasepencil {

/* -------------------------------------------------------------------- */
/** \name Interpolate Operator
 * \{ */

enum class GreasePencilInterpolateFlipMode : int8_t {
  /* No flip. */
  None = 0,
  /* Flip always. */
  Flip,
  /* Flip if needed. */
  FlipAuto,
};

enum class GreasePencilInterpolateLayerMode : int8_t {
  /* Only interpolate on the active layer. */
  Active = 0,
  /* Interpolate strokes on every layer. */
  All,
};

constexpr const float interpolate_factor_min = -1.0f;
constexpr const float interpolate_factor_max = 2.0f;

/* Pair of curves in a layer that get interpolated. */
struct InterpolationPairs {
  Vector<int> from_frames;
  Vector<int> to_frames;
  Vector<int> from_curves;
  Vector<int> to_curves;
};

struct GreasePencilInterpolateOpData {
  struct LayerData {
    /* Curve pairs to interpolate from this layer. */
    InterpolationPairs curve_pairs;

    /* Geometry of the target frame before interpolation for restoring on cancel. */
    // TODO
    // bke::CurvesGeometry stored_curves;
  };

  /* Layers to include. */
  IndexMaskMemory layer_mask_memory;
  IndexMask layer_mask;
  /* Interpolate only selected keyframes. */
  bool interpolate_selected_only;
  /* Exclude breakdown keyframes when finding intervals. */
  bool exclude_breakdowns;

  /* Interpolation factor bias controlled by the user. */
  float shift;
  /* Interpolation base factor for the active layer. */
  float init_factor;
  GreasePencilInterpolateFlipMode flipmode;
  float smooth_factor;
  int smooth_steps;

  NumInput numeric_input;
  Array<LayerData> layer_data;
  int active_layer_index;
};

using FramesMapKeyInterval = std::pair<int, int>;

static std::optional<FramesMapKeyInterval> find_frames_interval(
    const bke::greasepencil::Layer &layer, const int frame_number, const bool exclude_breakdowns)
{
  using bke::greasepencil::FramesMapKey;
  using SortedKeysIterator = Span<FramesMapKey>::iterator;

  const Span<FramesMapKey> sorted_keys = layer.sorted_keys();
  SortedKeysIterator next_key_it = std::upper_bound(
      sorted_keys.begin(), sorted_keys.end(), frame_number);
  if (next_key_it == sorted_keys.end() || next_key_it == sorted_keys.begin()) {
    return std::nullopt;
  }
  SortedKeysIterator prev_key_it = next_key_it - 1;

  /* Skip over invalid keyframes on either side. */
  auto is_valid_keyframe = [&](const FramesMapKey key) {
    const GreasePencilFrame &frame = *layer.frame_at(key);
    if (frame.is_end()) {
      return false;
    }
    if (exclude_breakdowns && frame.type == BEZT_KEYTYPE_BREAKDOWN) {
      return false;
    }
    return true;
  };

  for (; next_key_it != sorted_keys.end(); ++next_key_it) {
    if (is_valid_keyframe(*next_key_it)) {
      break;
    }
  }
  for (; prev_key_it != sorted_keys.begin(); --prev_key_it) {
    if (is_valid_keyframe(*prev_key_it)) {
      break;
    }
  }
  if (next_key_it == sorted_keys.end() || !is_valid_keyframe(*prev_key_it)) {
    return std::nullopt;
  }

  return std::make_pair(*prev_key_it, *next_key_it);
}

/* Build index lists for curve interpolation using index. */
static void find_curve_mapping_from_index(const GreasePencil &grease_pencil,
                                          const bke::greasepencil::Layer &layer,
                                          const int current_frame,
                                          const bool exclude_breakdowns,
                                          InterpolationPairs &pairs)
{
  using bke::greasepencil::Drawing;

  const std::optional<FramesMapKeyInterval> interval = find_frames_interval(
      layer, current_frame, exclude_breakdowns);
  if (!interval) {
    return;
  }

  BLI_assert(layer.has_drawing_at(interval->first));
  BLI_assert(layer.has_drawing_at(interval->second));
  const Drawing &from_drawing = *grease_pencil.get_drawing_at(layer, interval->first);
  const Drawing &to_drawing = *grease_pencil.get_drawing_at(layer, interval->second);

  const int pairs_num = std::min(from_drawing.strokes().curves_num(),
                                 to_drawing.strokes().curves_num());

  const int old_pairs_num = pairs.from_frames.size();
  pairs.from_frames.append_n_times(interval->first, pairs_num);
  pairs.to_frames.append_n_times(interval->first, pairs_num);
  pairs.from_curves.reinitialize(old_pairs_num + pairs_num);
  pairs.to_curves.reinitialize(old_pairs_num + pairs_num);
  array_utils::fill_index_range(
      pairs.from_curves.as_mutable_span().slice(old_pairs_num, pairs_num));
  array_utils::fill_index_range(pairs.to_curves.as_mutable_span().slice(old_pairs_num, pairs_num));
}

/* Build index lists for curve interpolation between two frames. */
static void find_curve_mapping_from_selection_order(const bke::greasepencil::Layer &layer,
                                                    InterpolationPairs &pairs)
{
  const bke::greasepencil::OrderedSelection &global_selection = layer.runtime->ordered_selection;

  std::cout << "Global Selection:" << std::endl;
  for (const int i : global_selection.data().index_range()) {
    std::cout << "  Frame " << global_selection.data()[i].frame_number << " Stroke "
              << global_selection.data()[i].stroke_index << std::endl;
  }

  const int pairs_num = global_selection.data().size() / 2;
  pairs.from_frames.reinitialize(pairs_num);
  pairs.to_frames.reinitialize(pairs_num);
  pairs.from_curves.reinitialize(pairs_num);
  pairs.to_curves.reinitialize(pairs_num);
  for (const int i : IndexRange(pairs_num)) {
    // TODO should do a range check and clamp here.
    pairs.from_frames[i] = global_selection.data()[2 * i].frame_number;
    pairs.to_frames[i] = global_selection.data()[2 * i + 1].frame_number;
    pairs.from_curves[i] = global_selection.data()[2 * i].stroke_index;
    pairs.to_curves[i] = global_selection.data()[2 * i + 1].stroke_index;
  }
}

static bke::CurvesGeometry interpolate_between_curves(const GreasePencil &grease_pencil,
                                                      const bke::greasepencil::Layer &layer,
                                                      const InterpolationPairs &curve_pairs,
                                                      const float mix_factor)
{
  using namespace blender;
  using bke::greasepencil::Drawing;

  const int dst_curve_num = curve_pairs.from_curves.size();
  BLI_assert(curve_pairs.to_curves.size() == dst_curve_num);
  BLI_assert(curve_pairs.from_frames.size() == dst_curve_num);
  BLI_assert(curve_pairs.to_frames.size() == dst_curve_num);

  /* Sort pairs by unique to/from frame combinations.
   * Curves for each frame pair are then interpolated together.
   * Map entries are indices into the original curve_pairs array,
   * so the order of strokes can be maintained. */
  Array<int> sorted_pairs(dst_curve_num);
  array_utils::fill_index_range(sorted_pairs.as_mutable_span());
  std::sort(sorted_pairs.begin(), sorted_pairs.end(), [&](const int a, const int b) {
    const int from_frame_a = curve_pairs.from_frames[a];
    const int to_frame_a = curve_pairs.to_frames[a];
    const int from_frame_b = curve_pairs.from_frames[b];
    const int to_frame_b = curve_pairs.to_frames[b];
    return from_frame_a < from_frame_b ||
           (from_frame_a == from_frame_b && to_frame_a < to_frame_b);
  });
  /* Find ranges of sorted pairs with the same from/to frame intervals. */
  Vector<int> pair_offsets;
  const OffsetIndices curves_by_pair = [&]() {
    int prev_from_frame = INT_MIN;
    int prev_to_frame = INT_MIN;
    int current_count = 0;
    for (const int sorted_index : IndexRange(dst_curve_num)) {
      const int pair_index = sorted_pairs[sorted_index];
      const int from_frame = curve_pairs.from_frames[pair_index];
      const int to_frame = curve_pairs.to_frames[pair_index];
      if (from_frame != prev_from_frame || to_frame != prev_to_frame) {
        /* New pair. */
        if (current_count > 0) {
          pair_offsets.append(current_count);
        }
        current_count = 0;
      }
    }
    /* Last entry for overall size. */
    if (pair_offsets.is_empty()) {
      return OffsetIndices<int>{};
    }

    pair_offsets.append(0);
    return offset_indices::accumulate_counts_to_offsets(pair_offsets);
  }();

  /* Compute curve length for each pair. */
  Vector<int> dst_curve_offsets;
  const OffsetIndices dst_points_by_curve = [&]() {
    for (const int pair_range_i : curves_by_pair.index_range()) {
      const IndexRange pair_range = curves_by_pair[pair_range_i];
      BLI_assert(!pair_range.is_empty());

      const int first_pair_index = sorted_pairs[pair_range.first()];
      const int from_frame = curve_pairs.from_frames[first_pair_index];
      const int to_frame = curve_pairs.to_frames[first_pair_index];
      const Drawing *from_drawing = grease_pencil.get_drawing_at(layer, from_frame);
      const Drawing *to_drawing = grease_pencil.get_drawing_at(layer, to_frame);
      if (!from_drawing || !to_drawing) {
        continue;
      }

      const OffsetIndices from_points_by_curve = from_drawing->strokes().points_by_curve();
      const OffsetIndices to_points_by_curve = to_drawing->strokes().points_by_curve();
      for (const int sorted_index : pair_range) {
        const int pair_index = sorted_pairs[sorted_index];
        const int from_curve = curve_pairs.from_curves[pair_index];
        const int to_curve = curve_pairs.to_curves[pair_index];

        dst_curve_offsets[pair_index] = std::max(from_points_by_curve[from_curve].size(),
                                                 to_points_by_curve[to_curve].size());
      }
    }
    /* Last entry for overall size. */
    if (dst_curve_offsets.is_empty()) {
      return OffsetIndices<int>{};
    }

    dst_curve_offsets.append(0);
    return offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);
  }();
  const int dst_point_num = dst_points_by_curve.total_size();

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  /* Offsets are empty when there are no curves. */
  if (dst_curve_num > 0) {
    dst_curves.offsets_for_write().copy_from(dst_curve_offsets);
  }

  for (const int pair_range_i : curves_by_pair.index_range()) {
    const IndexRange pair_range = curves_by_pair[pair_range_i];
    const int first_pair_index = sorted_pairs[pair_range.first()];
    const int from_frame = curve_pairs.from_frames[first_pair_index];
    const int to_frame = curve_pairs.to_frames[first_pair_index];
    const Drawing *from_drawing = grease_pencil.get_drawing_at(layer, from_frame);
    const Drawing *to_drawing = grease_pencil.get_drawing_at(layer, to_frame);
    if (!from_drawing || !to_drawing) {
      continue;
    }

    /* Subset of target curves that are filled by this frame pair. */
    IndexMaskMemory selection_memory;
    const IndexMask selection = IndexMask::from_indices(sorted_pairs.as_span().slice(pair_range),
                                                        selection_memory);

    geometry::interpolate_curves(
        from_drawing->strokes(), to_drawing->strokes(), selection, mix_factor, dst_curves);
  }

  dst_curves.tag_topology_changed();
  return dst_curves;
}

// /* Helper: Update all strokes interpolated */
// static void gpencil_interpolate_update_strokes(bContext *C, tGPDinterpolate *tgpi)
// {
//   bGPdata *gpd = tgpi->gpd;
//   const float shift = tgpi->shift;

//   LISTBASE_FOREACH (tGPDinterpolate_layer *, tgpil, &tgpi->ilayers) {
//     const float factor = tgpil->factor + shift;

//     bGPDframe *gpf = tgpil->gpl->actframe;
//     /* Free temp strokes used for display. */
//     gpencil_interpolate_free_tagged_strokes(gpf);

//     /* Clear previous interpolations. */
//     gpencil_interpolate_free_tagged_strokes(tgpil->interFrame);

//     LISTBASE_FOREACH (LinkData *, link, &tgpil->selected_strokes) {
//       bGPDstroke *gps_from = static_cast<bGPDstroke *>(link->data);
//       if (!BLI_ghash_haskey(tgpil->pair_strokes, gps_from)) {
//         continue;
//       }
//       bGPDstroke *gps_to = (bGPDstroke *)BLI_ghash_lookup(tgpil->pair_strokes, gps_from);

//       /* Create new stroke. */
//       bGPDstroke *new_stroke = BKE_gpencil_stroke_duplicate(gps_from, true, true);
//       new_stroke->flag |= GP_STROKE_TAG;
//       new_stroke->select_index = 0;

//       /* Update points position. */
//       gpencil_interpolate_update_points(gps_from, gps_to, new_stroke, factor);

//       /* Calc geometry data. */
//       BKE_gpencil_stroke_geometry_update(gpd, new_stroke);
//       /* Add to strokes. */
//       BLI_addtail(&tgpil->interFrame->strokes, new_stroke);

//       /* Add temp strokes to display. */
//       if (gpf) {
//         bGPDstroke *gps_eval = BKE_gpencil_stroke_duplicate(new_stroke, true, true);
//         gps_eval->flag |= GP_STROKE_TAG;
//         BLI_addtail(&gpf->strokes, gps_eval);
//       }
//     }
//   }

//   DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
//   WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
// }

// /* Helper: Create internal strokes interpolated */
// static void gpencil_interpolate_set_points(bContext *C, tGPDinterpolate *tgpi)
// {
//   Scene *scene = tgpi->scene;
//   bGPdata *gpd = tgpi->gpd;
//   bGPDlayer *active_gpl = CTX_data_active_gpencil_layer(C);
//   bGPDframe *actframe = active_gpl->actframe;
//   const bool exclude_breakdowns = (tgpi->flag & GP_TOOLFLAG_INTERPOLATE_EXCLUDE_BREAKDOWNS) !=
//   0;

//   /* save initial factor for active layer to define shift limits */
//   tgpi->init_factor = float(tgpi->cframe - actframe->framenum) /
//                       (actframe->next->framenum - actframe->framenum + 1);

//   /* limits are 100% below 0 and 100% over the 100% */
//   tgpi->low_limit = -1.0f - tgpi->init_factor;
//   tgpi->high_limit = 2.0f - tgpi->init_factor;

//   /* set layers */
//   LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
//     tGPDinterpolate_layer *tgpil;
//     /* all layers or only active */
//     if (!(tgpi->flag & GP_TOOLFLAG_INTERPOLATE_ALL_LAYERS) && (gpl != active_gpl)) {
//       continue;
//     }
//     /* only editable and visible layers are considered */
//     if (!BKE_gpencil_layer_is_editable(gpl) || (gpl->actframe == nullptr)) {
//       continue;
//     }
//     if ((gpl->actframe == nullptr) || (gpl->actframe->next == nullptr)) {
//       continue;
//     }

//     bGPDframe *gpf_prv = gpencil_get_previous_keyframe(gpl, scene->r.cfra, exclude_breakdowns);
//     if (gpf_prv == nullptr) {
//       continue;
//     }
//     bGPDframe *gpf_next = gpencil_get_next_keyframe(gpl, scene->r.cfra, exclude_breakdowns);
//     if (gpf_next == nullptr) {
//       continue;
//     }

//     /* Create temp data for each layer. */
//     tgpil = static_cast<tGPDinterpolate_layer *>(
//         MEM_callocN(sizeof(tGPDinterpolate_layer), "GPencil Interpolate Layer"));
//     tgpil->gpl = gpl;
//     tgpil->prevFrame = BKE_gpencil_frame_duplicate(gpf_prv, true);
//     tgpil->nextFrame = BKE_gpencil_frame_duplicate(gpf_next, true);

//     BLI_addtail(&tgpi->ilayers, tgpil);

//     /* Create a new temporary frame. */
//     tgpil->interFrame = static_cast<bGPDframe *>(MEM_callocN(sizeof(bGPDframe), "bGPDframe"));
//     tgpil->interFrame->framenum = tgpi->cframe;

//     /* get interpolation factor by layer (usually must be equal for all layers, but not sure) */
//     tgpil->factor = float(tgpi->cframe - tgpil->prevFrame->framenum) /
//                     (tgpil->nextFrame->framenum - tgpil->prevFrame->framenum + 1);

//     /* Load the relationship between frames. */
//     gpencil_stroke_pair_table(C, tgpi, tgpil);

//     /* Create new strokes data with interpolated points reading original stroke. */
//     LISTBASE_FOREACH (LinkData *, link, &tgpil->selected_strokes) {
//       bGPDstroke *gps_from = static_cast<bGPDstroke *>(link->data);
//       if (!BLI_ghash_haskey(tgpil->pair_strokes, gps_from)) {
//         continue;
//       }
//       bGPDstroke *gps_to = (bGPDstroke *)BLI_ghash_lookup(tgpil->pair_strokes, gps_from);

//       /* If destination stroke is smaller, resize new_stroke to size of gps_to stroke. */
//       if (gps_from->totpoints > gps_to->totpoints) {
//         BKE_gpencil_stroke_uniform_subdivide(gpd, gps_to, gps_from->totpoints, true);
//       }
//       if (gps_to->totpoints > gps_from->totpoints) {
//         BKE_gpencil_stroke_uniform_subdivide(gpd, gps_from, gps_to->totpoints, true);
//       }

//       /* Flip stroke. */
//       if (tgpi->flipmode == GP_INTERPOLATE_FLIP) {
//         BKE_gpencil_stroke_flip(gps_to);
//       }
//       else if (tgpi->flipmode == GP_INTERPOLATE_FLIPAUTO) {
//         if (gpencil_stroke_need_flip(tgpi->depsgraph, tgpi->ob, gpl, &tgpi->gsc, gps_from,
//         gps_to))
//         {
//           BKE_gpencil_stroke_flip(gps_to);
//         }
//       }

//       /* Create new stroke. */
//       bGPDstroke *new_stroke = BKE_gpencil_stroke_duplicate(gps_from, true, true);
//       new_stroke->flag |= GP_STROKE_TAG;
//       new_stroke->select_index = 0;

//       /* Update points position. */
//       gpencil_interpolate_update_points(gps_from, gps_to, new_stroke, tgpil->factor);
//       BKE_gpencil_stroke_smooth(new_stroke,
//                                 tgpi->smooth_factor,
//                                 tgpi->smooth_steps,
//                                 true,
//                                 true,
//                                 false,
//                                 false,
//                                 true,
//                                 nullptr);

//       /* Calc geometry data. */
//       BKE_gpencil_stroke_geometry_update(gpd, new_stroke);
//       /* add to strokes */
//       BLI_addtail(&tgpil->interFrame->strokes, new_stroke);
//     }
//   }
// }

static void grease_pencil_interpolate_status_indicators(
    bContext &C, const GreasePencilInterpolateOpData &opdata)
{
  Scene &scene = *CTX_data_scene(&C);
  ScrArea &area = *CTX_wm_area(&C);

  const StringRef msg = IFACE_("GPencil Interpolation: ");

  std::string status;
  if (hasNumInput(&opdata.numeric_input)) {
    char str_ofs[NUM_STR_REP_LEN];
    outputNumInput(&const_cast<NumInput &>(opdata.numeric_input), str_ofs, &scene.unit);
    status = msg + std::string(str_ofs);
  }
  else {
    status = msg + std::to_string(int((opdata.init_factor + opdata.shift) * 100.0f)) + " %";
  }

  ED_area_status_text(&area, status.c_str());
  ED_workspace_status_text(
      &C, IFACE_("ESC/RMB to cancel, Enter/LMB to confirm, WHEEL/MOVE to adjust factor"));
}

static bke::greasepencil::Drawing *get_or_create_drawing_at_frame(GreasePencil &grease_pencil,
                                                                  bke::greasepencil::Layer &layer,
                                                                  const int frame_number)
{
  using bke::greasepencil::FramesMapKey;

  std::optional<FramesMapKey> frame_key = layer.frame_key_at(frame_number);
  if (frame_key && *frame_key == frame_number) {
    return grease_pencil.get_drawing_at(layer, frame_number);
  }

  return grease_pencil.insert_frame(layer, frame_number);
}

static void grease_pencil_interpolate_update(bContext &C, const wmOperator &op)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  const auto &opdata = *static_cast<GreasePencilInterpolateOpData *>(op.customdata);
  const Scene &scene = *CTX_data_scene(&C);
  const int current_frame = scene.r.cfra;
  Object &object = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object.data);

  opdata.layer_mask.foreach_index([&](const int layer_index) {
    Layer &layer = *grease_pencil.layers_for_write()[layer_index];
    const GreasePencilInterpolateOpData::LayerData &layer_data = opdata.layer_data[layer_index];

    Drawing *dst_drawing = get_or_create_drawing_at_frame(grease_pencil, layer, current_frame);
    if (dst_drawing == nullptr) {
      return;
    }

    const float mix_factor = opdata.init_factor + opdata.shift;
    const bke::CurvesGeometry interpolated_curves = interpolate_between_curves(
        grease_pencil, layer, layer_data.curve_pairs, mix_factor);

    dst_drawing->strokes_for_write() = std::move(interpolated_curves);
    dst_drawing->tag_topology_changed();
  });

  grease_pencil_interpolate_status_indicators(C, opdata);

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(&C, NC_GPENCIL | NA_EDITED, nullptr);
}

static bool grease_pencil_interpolate_init(const bContext &C, wmOperator &op)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  const Scene &scene = *CTX_data_scene(&C);
  const int current_frame = scene.r.cfra;
  const Object &object = *CTX_data_active_object(&C);
  const GreasePencil &grease_pencil = *static_cast<const GreasePencil *>(object.data);

  BLI_assert(grease_pencil.has_active_layer());

  op.customdata = MEM_new<GreasePencilInterpolateOpData>(__func__);
  GreasePencilInterpolateOpData &data = *static_cast<GreasePencilInterpolateOpData *>(
      op.customdata);

  data.shift = RNA_float_get(op.ptr, "shift");
  data.interpolate_selected_only = RNA_boolean_get(op.ptr, "interpolate_selected_only");
  data.exclude_breakdowns = RNA_boolean_get(op.ptr, "exclude_breakdowns");
  data.flipmode = GreasePencilInterpolateFlipMode(RNA_enum_get(op.ptr, "flip"));
  data.smooth_factor = RNA_float_get(op.ptr, "smooth_factor");
  data.smooth_steps = RNA_int_get(op.ptr, "smooth_steps");
  data.active_layer_index = *grease_pencil.get_layer_index(*grease_pencil.get_active_layer());

  const auto layer_mode = GreasePencilInterpolateLayerMode(RNA_enum_get(op.ptr, "layers"));
  switch (layer_mode) {
    case GreasePencilInterpolateLayerMode::Active:
      data.layer_mask = IndexRange::from_single(data.active_layer_index);
      break;
    case GreasePencilInterpolateLayerMode::All:
      data.layer_mask = IndexMask::from_predicate(
          grease_pencil.layers().index_range(),
          GrainSize(1024),
          data.layer_mask_memory,
          [&](const int layer_index) {
            return grease_pencil.layers()[layer_index]->is_editable();
          });
      break;
  }

  data.layer_data.reinitialize(grease_pencil.layers().size());
  data.layer_mask.foreach_index([&](const int layer_index) {
    const Layer &layer = *grease_pencil.layers()[layer_index];
    GreasePencilInterpolateOpData::LayerData &layer_data = data.layer_data[layer_index];

    if (data.interpolate_selected_only) {
      find_curve_mapping_from_selection_order(layer, layer_data.curve_pairs);
    }
    else {
      /* Pair from/to curves by index. */
      find_curve_mapping_from_index(
          grease_pencil, layer, current_frame, data.exclude_breakdowns, layer_data.curve_pairs);
    }
  });

  // const GreasePencilInterpolateOpData::LayerData &active_layer_data =
  //     data.layer_data[data.active_layer_index];
  // if (active_layer_data.from_frame_number == active_layer_data.to_frame_number) {
  //   BKE_report(
  //       op.reports,
  //       RPT_ERROR,
  //       "Cannot find valid keyframes to interpolate (Breakdowns keyframes are not allowed)");
  //   MEM_delete(&data);
  //   op.customdata = nullptr;
  //   return false;
  // }
  // data.init_factor = float(current_frame - active_layer_data.from_frame_number) /
  //                    (active_layer_data.to_frame_number - active_layer_data.from_frame_number +
  //                    1);

  return true;
}

/* Exit and free memory. */
static void grease_pencil_interpolate_exit(bContext &C, wmOperator &op)
{
  ScrArea &area = *CTX_wm_area(&C);

  if (op.customdata == nullptr) {
    return;
  }

  ED_area_status_text(&area, nullptr);
  ED_workspace_status_text(&C, nullptr);

  MEM_delete(static_cast<GreasePencilInterpolateOpData *>(op.customdata));
  op.customdata = nullptr;

  // DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  // WM_event_add_notifier(&C, NC_GPENCIL | NA_EDITED, nullptr);
}

static bool grease_pencil_interpolate_poll(bContext *C)
{
  if (!ed::greasepencil::active_grease_pencil_poll(C)) {
    return false;
  }
  ToolSettings *ts = CTX_data_tool_settings(C);
  if (!ts || !ts->gp_paint) {
    return false;
  }
  /* Only 3D view */
  ScrArea *area = CTX_wm_area(C);
  if (area && area->spacetype != SPACE_VIEW3D) {
    return false;
  }

  return true;
}

/* Invoke handler: Initialize the operator */
static int grease_pencil_interpolate_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  wmWindow &win = *CTX_wm_window(C);

  if (!grease_pencil_interpolate_init(*C, *op)) {
    grease_pencil_interpolate_exit(*C, *op);
    return OPERATOR_CANCELLED;
  }
  GreasePencilInterpolateOpData &opdata = *static_cast<GreasePencilInterpolateOpData *>(
      op->customdata);

  /* Set cursor to indicate modal operator. */
  WM_cursor_modal_set(&win, WM_CURSOR_EW_SCROLL);

  grease_pencil_interpolate_status_indicators(*C, opdata);

  // DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);

  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

enum class InterpolateToolModalEvent : int8_t {
  Cancel = 1,
  Confirm,
  Increase,
  Decrease,
};

/* Modal handler: Events handling during interactive part */
static int grease_pencil_interpolate_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  wmWindow &win = *CTX_wm_window(C);
  const ARegion &region = *CTX_wm_region(C);
  ScrArea &area = *CTX_wm_area(C);
  GreasePencilInterpolateOpData &opdata = *static_cast<GreasePencilInterpolateOpData *>(
      op->customdata);
  const bool has_numinput = hasNumInput(&opdata.numeric_input);

  switch (event->type) {
    case EVT_MODAL_MAP: {
      switch (InterpolateToolModalEvent(event->val)) {
        case InterpolateToolModalEvent::Cancel:
          ED_area_status_text(&area, nullptr);
          ED_workspace_status_text(C, nullptr);
          WM_cursor_modal_restore(&win);

          grease_pencil_interpolate_exit(*C, *op);
          return OPERATOR_CANCELLED;
        case InterpolateToolModalEvent::Confirm:
          ED_area_status_text(&area, nullptr);
          ED_workspace_status_text(C, nullptr);
          WM_cursor_modal_restore(&win);

          //     /* insert keyframes as required... */
          //     LISTBASE_FOREACH (tGPDinterpolate_layer *, tgpil, &tgpi->ilayers) {
          //       gpf_dst = BKE_gpencil_layer_frame_get(tgpil->gpl, tgpi->cframe,
          //       GP_GETFRAME_ADD_NEW); gpf_dst->key_type = BEZT_KEYTYPE_BREAKDOWN;

          //       /* Copy strokes. */
          //       LISTBASE_FOREACH (bGPDstroke *, gps_src, &tgpil->interFrame->strokes) {
          //         if (gps_src->totpoints == 0) {
          //           continue;
          //         }

          //         /* make copy of source stroke, then adjust pointer to points too */
          //         gps_dst = BKE_gpencil_stroke_duplicate(gps_src, true, true);
          //         gps_dst->flag &= ~GP_STROKE_TAG;

          //         /* Calc geometry data. */
          //         BKE_gpencil_stroke_geometry_update(tgpi->gpd, gps_dst);

          //         BLI_addtail(&gpf_dst->strokes, gps_dst);
          //       }
          //     }

          /* Write current factor to properties for the next execution. */
          RNA_float_set(op->ptr, "shift", opdata.shift);

          grease_pencil_interpolate_exit(*C, *op);
          return OPERATOR_FINISHED;
        case InterpolateToolModalEvent::Increase:
          opdata.shift = std::clamp(opdata.init_factor + opdata.shift + 0.01f,
                                    interpolate_factor_min,
                                    interpolate_factor_max) -
                         opdata.init_factor;
          grease_pencil_interpolate_update(*C, *op);
          break;
        case InterpolateToolModalEvent::Decrease:
          opdata.shift = std::clamp(opdata.init_factor + opdata.shift - 0.01f,
                                    interpolate_factor_min,
                                    interpolate_factor_max) -
                         opdata.init_factor;
          grease_pencil_interpolate_update(*C, *op);
          break;
      }
      break;
    }
    case MOUSEMOVE:
      /* Only handle mouse-move if not doing numeric-input. */
      if (!has_numinput) {
        const float mouse_pos = event->mval[0];
        const float factor = std::clamp(
            mouse_pos / region.winx, interpolate_factor_min, interpolate_factor_max);
        opdata.shift = factor - opdata.init_factor;

        grease_pencil_interpolate_update(*C, *op);
      }
      break;
    default: {
      if ((event->val == KM_PRESS) && handleNumInput(C, &opdata.numeric_input, event)) {
        float value = (opdata.init_factor + opdata.shift) * 100.0f;
        applyNumInput(&opdata.numeric_input, &value);
        opdata.shift = std::clamp(value * 0.01f, interpolate_factor_min, interpolate_factor_max) -
                       opdata.init_factor;

        grease_pencil_interpolate_update(*C, *op);
        break;
      }
      /* Unhandled event, allow to pass through. */
      return OPERATOR_RUNNING_MODAL | OPERATOR_PASS_THROUGH;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static void grease_pencil_interpolate_cancel(bContext *C, wmOperator *op)
{
  grease_pencil_interpolate_exit(*C, *op);
}

static void GREASE_PENCIL_OT_interpolate(wmOperatorType *ot)
{
  static const EnumPropertyItem flip_modes[] = {
      {int(GreasePencilInterpolateFlipMode::None), "NONE", 0, "No Flip", ""},
      {int(GreasePencilInterpolateFlipMode::Flip), "FLIP", 0, "Flip", ""},
      {int(GreasePencilInterpolateFlipMode::FlipAuto), "AUTO", 0, "Automatic", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem gpencil_interpolation_layer_items[] = {
      {int(GreasePencilInterpolateLayerMode::Active), "ACTIVE", 0, "Active", ""},
      {int(GreasePencilInterpolateLayerMode::All), "ALL", 0, "All Layers", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  /* identifiers */
  ot->name = "Grease Pencil Interpolation";
  ot->idname = "GREASE_PENCIL_OT_interpolate";
  ot->description = "Interpolate grease pencil strokes between frames";

  /* callbacks */
  ot->invoke = grease_pencil_interpolate_invoke;
  ot->modal = grease_pencil_interpolate_modal;
  ot->cancel = grease_pencil_interpolate_cancel;
  ot->poll = grease_pencil_interpolate_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* properties */
  RNA_def_float_factor(
      ot->srna,
      "shift",
      0.0f,
      -1.0f,
      1.0f,
      "Shift",
      "Bias factor for which frame has more influence on the interpolated strokes",
      -0.9f,
      0.9f);

  RNA_def_enum(ot->srna,
               "layers",
               gpencil_interpolation_layer_items,
               0,
               "Layer",
               "Layers included in the interpolation");

  RNA_def_boolean(ot->srna,
                  "interpolate_selected_only",
                  false,
                  "Only Selected",
                  "Interpolate only selected strokes");

  RNA_def_boolean(ot->srna,
                  "exclude_breakdowns",
                  false,
                  "Exclude Breakdowns",
                  "Exclude existing Breakdowns keyframes as interpolation extremes");

  RNA_def_enum(ot->srna,
               "flip",
               flip_modes,
               int(GreasePencilInterpolateFlipMode::FlipAuto),
               "Flip Mode",
               "Invert destination stroke to match start and end with source stroke");

  RNA_def_int(ot->srna,
              "smooth_steps",
              1,
              1,
              3,
              "Iterations",
              "Number of times to smooth newly created strokes",
              1,
              3);

  RNA_def_float(ot->srna,
                "smooth_factor",
                0.0f,
                0.0f,
                2.0f,
                "Smooth",
                "Amount of smoothing to apply to interpolated strokes, to reduce jitter/noise",
                0.0f,
                2.0f);
}

/** \} */

}  // namespace blender::ed::sculpt_paint::greasepencil

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_operatortypes_grease_pencil_interpolate()
{
  using namespace blender::ed::sculpt_paint::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_interpolate);
}

void ED_interpolatetool_modal_keymap(wmKeyConfig *keyconf)
{
  using namespace blender::ed::sculpt_paint::greasepencil;
  static const EnumPropertyItem modal_items[] = {
      {int(InterpolateToolModalEvent::Cancel), "CANCEL", 0, "Cancel", ""},
      {int(InterpolateToolModalEvent::Confirm), "CONFIRM", 0, "Confirm", ""},
      {int(InterpolateToolModalEvent::Increase), "INCREASE", 0, "Increase", ""},
      {int(InterpolateToolModalEvent::Decrease), "DECREASE", 0, "Decrease", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "Interpolate Tool Modal Map");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "Interpolate Tool Modal Map", modal_items);

  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_interpolate");
}

/** \} */
