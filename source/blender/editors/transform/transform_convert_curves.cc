/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include <optional>

#include "BLI_array.hh"
#include "BLI_inplace_priority_queue.hh"
#include "BLI_math_matrix.h"
#include "BLI_span.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"

#include "ED_curves.hh"

#include "MEM_guardedalloc.h"

#include "transform.hh"
#include "transform_convert.hh"

/* -------------------------------------------------------------------- */
/** \name Curve/Surfaces Transform Creation
 * \{ */

namespace blender::ed::transform::curves {

static void calculate_curve_point_distances_for_proportional_editing(
    const Span<float3> positions, MutableSpan<float> r_distances)
{
  Array<bool, 32> visited(positions.size(), false);

  InplacePriorityQueue<float, std::less<float>> queue(r_distances);
  while (!queue.is_empty()) {
    int64_t index = queue.pop_index();
    if (visited[index]) {
      continue;
    }
    visited[index] = true;

    /* TODO(Falk): Handle cyclic curves here. */
    if (index > 0 && !visited[index - 1]) {
      int adjacent = index - 1;
      float dist = r_distances[index] + math::distance(positions[index], positions[adjacent]);
      if (dist < r_distances[adjacent]) {
        r_distances[adjacent] = dist;
        queue.priority_changed(adjacent);
      }
    }
    if (index < positions.size() - 1 && !visited[index + 1]) {
      int adjacent = index + 1;
      float dist = r_distances[index] + math::distance(positions[index], positions[adjacent]);
      if (dist < r_distances[adjacent]) {
        r_distances[adjacent] = dist;
        queue.priority_changed(adjacent);
      }
    }
  }
}

static void createTransCurvesVerts(bContext * /*C*/, TransInfo *t)
{
  MutableSpan<TransDataContainer> trans_data_contrainers(t->data_container, t->data_container_len);
  IndexMaskMemory memory;
  Array<std::array<IndexMask, 3>> selection_per_attribute(t->data_container_len);
  Array<int> selection_counts(t->data_container_len);
  Array<IndexMask> bezier_curves(t->data_container_len);
  const bool use_proportional_edit = (t->flag & T_PROP_EDIT_ALL) != 0;
  const bool use_connected_only = (t->flag & T_PROP_CONNECTED) != 0;

  Vector<bool> must_be_selected;

  /* Count selected elements per object and create TransData structs. */
  for (const int i : trans_data_contrainers.index_range()) {
    TransDataContainer &tc = trans_data_contrainers[i];
    Curves *curves_id = static_cast<Curves *>(tc.obedit->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    const OffsetIndices<int> points_by_curve = curves.points_by_curve();
    const VArray<int8_t> handle_types_left = curves.handle_types_left();
    const VArray<int8_t> handle_types_right = curves.handle_types_right();
    Span<StringRef> selection_attribute_names = ed::curves::get_curves_selection_attribute_names(
        curves);
    selection_counts[i] = selection_attribute_names.size();
    for (const int attribute_i : selection_attribute_names.index_range()) {
      const StringRef &selection_name = selection_attribute_names[attribute_i];
      selection_per_attribute[i][attribute_i] = ed::curves::retrieve_selected_points(
          curves, selection_name, memory);
    }

    bezier_curves[i] = bke::curves::indices_for_type(curves.curve_types(),
                                                     curves.curve_type_counts(),
                                                     CURVE_TYPE_BEZIER,
                                                     curves.curves_range(),
                                                     memory);
    /* Alter selection as in legacy curves bezt_select_to_transform_triple_flag(). */
    if (bezier_curves[i].size()) {
      must_be_selected.reinitialize(curves.points_num());
      bezier_curves[i].foreach_index(GrainSize(128), [&](const int bezier_index) {
        for (const int point_i : points_by_curve[bezier_index]) {
          if (selection_per_attribute[i][0].contains(point_i)) {
            const HandleType type_left = HandleType(handle_types_left[point_i]);
            const HandleType type_right = HandleType(handle_types_right[point_i]);
            if (ELEM(type_left, BEZIER_HANDLE_AUTO, BEZIER_HANDLE_ALIGN) &&
                ELEM(type_right, BEZIER_HANDLE_AUTO, BEZIER_HANDLE_ALIGN))
            {
              must_be_selected[point_i] = true;
            }
          }
        }
      });
      IndexMask must_be_selected_mask = IndexMask::from_bools(must_be_selected.as_span(), memory);
      if (must_be_selected.size()) {
        selection_per_attribute[i][1] = IndexMask::from_union(
            selection_per_attribute[i][1], must_be_selected_mask, memory);
        selection_per_attribute[i][2] = IndexMask::from_union(
            selection_per_attribute[i][2], must_be_selected_mask, memory);
      }
    }

    if (use_proportional_edit) {
      Array<int> bezier_point_offset_data(bezier_curves[i].size() + 1);
      OffsetIndices<int> bezier_offsets = offset_indices::gather_selected_offsets(
          curves.points_by_curve(), bezier_curves[i], bezier_point_offset_data);

      const int bezier_point_count = bezier_offsets.total_size();
      tc.data_len = curves.points_num() + 2 * bezier_point_count;
    }
    else {
      tc.data_len = std::reduce(
          selection_per_attribute[i].begin(),
          selection_per_attribute[i].end(),
          int(0),
          [&](int acc, const IndexMask &selected) { return selected.size() + acc; });
    }

    if (tc.data_len > 0) {
      tc.data = MEM_cnew_array<TransData>(tc.data_len, __func__);
    }
  }

  /* Populate TransData structs. */
  for (const int i : trans_data_contrainers.index_range()) {
    TransDataContainer &tc = trans_data_contrainers[i];
    if (tc.data_len == 0) {
      continue;
    }
    Object *object = tc.obedit;
    Curves *curves_id = static_cast<Curves *>(object->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();

    std::optional<MutableSpan<float>> value_attribute;
    bke::SpanAttributeWriter<float> attribute_writer;
    if (t->mode == TFM_CURVE_SHRINKFATTEN) {
      bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
      attribute_writer = attributes.lookup_or_add_for_write_span<float>(
          "radius",
          bke::AttrDomain::Point,
          bke::AttributeInitVArray(VArray<float>::ForSingle(0.01f, curves.points_num())));
      value_attribute = attribute_writer.span;
    }
    else if (t->mode == TFM_TILT) {
      bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
      attribute_writer = attributes.lookup_or_add_for_write_span<float>("tilt",
                                                                        bke::AttrDomain::Point);
      value_attribute = attribute_writer.span;
    }

    int64_t trans_data_offset = 0;
    Span<IndexMask> selections_per_attribute(selection_per_attribute[i].data(),
                                             selection_counts[i]);
    curve_populate_trans_data_structs(tc,
                                      curves,
                                      object->object_to_world(),
                                      value_attribute,
                                      selections_per_attribute,
                                      use_proportional_edit,
                                      curves.curves_range(),
                                      use_connected_only,
                                      trans_data_offset,
                                      bezier_curves[i]);
    for (const IndexMask selection : selections_per_attribute) {
      trans_data_offset += use_proportional_edit ? 0 : selection.size();
    }

    /* TODO: This is wrong. The attribute writer should live at least as long as the span. */
    attribute_writer.finish();
  }
}

static void recalcData_curves(TransInfo *t)
{
  const Span<TransDataContainer> trans_data_contrainers(t->data_container, t->data_container_len);
  for (const TransDataContainer &tc : trans_data_contrainers) {
    Curves *curves_id = static_cast<Curves *>(tc.obedit->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    if (t->mode == TFM_CURVE_SHRINKFATTEN) {
      /* No cache to update currently. */
    }
    else if (t->mode == TFM_TILT) {
      curves.tag_normals_changed();
    }
    else {
      curves.tag_positions_changed();
      // curves.calculate_bezier_auto_handles();
    }
    DEG_id_tag_update(&curves_id->id, ID_RECALC_GEOMETRY);
  }
}

}  // namespace blender::ed::transform::curves

void curve_populate_trans_data_structs(TransDataContainer &tc,
                                       blender::bke::CurvesGeometry &curves,
                                       const blender::float4x4 &transform,
                                       std::optional<blender::MutableSpan<float>> value_attribute,
                                       const blender::Span<blender::IndexMask> selected_indices,
                                       const bool use_proportional_edit,
                                       const blender::IndexMask &affected_curves,
                                       bool use_connected_only,
                                       int trans_data_offset,
                                       const blender::IndexMask &bezier_curves)
{
  using namespace blender;
  /* Term layer in function is used to refer Bezier curve points, left and right handles as layers
   * 0, 1 and 2 respectively. For given Bezier curve in layers with points [P0, P1, P2], left
   * handles [L0, L1, L2] and [R0, R1, R2] flattened  version version will be [L0, P0, R0, L1, P1,
   * R1, L2, P2, R2]. */
  static const Array<int> layer_shift_in_bezier = {1, 0, 2};
  static const Array<int> layer_shift_in_others = {0};

  const std::array<MutableSpan<float3>, 3> positions_per_layer{
      curves.positions_for_write(),
      curves.handle_positions_left_for_write(),
      curves.handle_positions_right_for_write()};

  float mtx[3][3], smtx[3][3];
  copy_m3_m4(mtx, transform.ptr());
  pseudoinverse_m3_m3(smtx, mtx, PSEUDOINVERSE_EPSILON);

  if (use_proportional_edit) {
    const OffsetIndices<int> points_by_curve = curves.points_by_curve();
    const VArray<int8_t> curve_types = curves.curve_types();
    Array<int> flat_offset_data(points_by_curve.size() + 1);
    const OffsetIndices<int> flat_points_by_curve = expand_selected_offsets(
        points_by_curve, bezier_curves, 3, flat_offset_data);
    affected_curves.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
      Vector<float> closest_distances;
      Vector<float3> all_curve_positions;

      for (const int curve_i : segment) {
        const IndexRange points = points_by_curve[curve_i];
        const IndexRange all_curve_points = flat_points_by_curve[curve_i];
        const int selected_point_count = std::reduce(selected_indices.begin(),
                                                     selected_indices.end(),
                                                     int64_t(0),
                                                     [&](int64_t acc, const IndexMask &mask) {
                                                       return mask.slice_content(points).size() +
                                                              acc;
                                                     });
        const bool has_any_selected = selected_point_count > 0;
        if (!has_any_selected && use_connected_only) {
          for (const int point_i : all_curve_points) {
            TransData &td = tc.data[point_i + trans_data_offset];
            td.flag |= TD_SKIP;
          }
          continue;
        }

        closest_distances.reinitialize(all_curve_points.size());
        closest_distances.fill(std::numeric_limits<float>::max());
        all_curve_positions.reinitialize(all_curve_points.size());

        IndexRange layers_range = IndexRange(curve_types[curve_i] == CURVE_TYPE_BEZIER ? 3 : 1);

        Span<int> layer_shift = (curve_types[curve_i] == CURVE_TYPE_BEZIER) ?
                                    layer_shift_in_bezier :
                                    layer_shift_in_others;

        for (const int layer : layers_range) {
          for (const int i : points.index_range()) {
            const int point_in_layer_i = points[i];
            const int flat_i = layer_shift[layer] + layers_range.size() * i;
            const int point_i = all_curve_points[flat_i];

            all_curve_positions[flat_i] = positions_per_layer[layer][point_in_layer_i];

            TransData &td = tc.data[point_i + trans_data_offset];
            float3 *elem = &positions_per_layer[layer][point_in_layer_i];

            copy_v3_v3(td.iloc, *elem);
            copy_v3_v3(td.center, td.iloc);
            td.loc = *elem;

            td.flag = 0;
            if (selected_indices[layer].contains(point_in_layer_i)) {
              closest_distances[flat_i] = 0.0f;
              td.flag = TD_SELECTED;
            }

            if (value_attribute && layer == 0) {
              float *value = &((*value_attribute)[point_in_layer_i]);
              td.val = value;
              td.ival = *value;
            }

            td.ext = nullptr;

            copy_m3_m3(td.smtx, smtx);
            copy_m3_m3(td.mtx, mtx);
          }
        }

        if (use_connected_only) {
          blender::ed::transform::curves::calculate_curve_point_distances_for_proportional_editing(
              all_curve_positions.as_span(), closest_distances.as_mutable_span());
          for (const int i : all_curve_points.index_range()) {
            TransData &td = tc.data[all_curve_points[i] + trans_data_offset];
            td.dist = closest_distances[i];
          }
        }
      }
    });
  }
  else {
    int layer_offset = 0;
    for (const int layer : selected_indices.index_range()) {
      MutableSpan<float3> positions = positions_per_layer[layer];
      const IndexMask &selection = selected_indices[layer];
      threading::parallel_for(selection.index_range(), 1024, [&](const IndexRange range) {
        for (const int selection_i : range) {
          TransData *td = &tc.data[selection_i + trans_data_offset + layer_offset];
          const int point_i = selection[selection_i];
          float3 *elem = &positions[point_i];

          copy_v3_v3(td->iloc, *elem);
          copy_v3_v3(td->center, td->iloc);
          td->loc = *elem;

          if (value_attribute) {
            float *value = &((*value_attribute)[point_i]);
            td->val = value;
            td->ival = *value;
          }

          td->flag = TD_SELECTED;
          td->ext = nullptr;

          copy_m3_m3(td->smtx, smtx);
          copy_m3_m3(td->mtx, mtx);
        }
      });
      layer_offset += selection.size();
    }
  }
}

/** \} */

TransConvertTypeInfo TransConvertType_Curves = {
    /*flags*/ (T_EDIT | T_POINTS),
    /*create_trans_data*/ blender::ed::transform::curves::createTransCurvesVerts,
    /*recalc_data*/ blender::ed::transform::curves::recalcData_curves,
    /*special_aftertrans_update*/ nullptr,
};
