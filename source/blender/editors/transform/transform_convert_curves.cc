/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "BLI_array.hh"
#include "BLI_index_mask_ops.hh"
#include "BLI_span.hh"

#include "BKE_curves.hh"

#include "ED_curves.h"

#include "MEM_guardedalloc.h"

#include "transform.h"
#include "transform_convert.h"

/* -------------------------------------------------------------------- */
/** \name Curve/Surfaces Transform Creation
 * \{ */

namespace blender::ed::transform::curves {

static void createTransCurvesVerts(bContext * /*C*/, TransInfo *t)
{
  MutableSpan<TransDataContainer> trans_data_contrainers(t->data_container, t->data_container_len);
  Array<Vector<int64_t>> selected_indices_per_object(t->data_container_len);
  Array<IndexMask> selection_per_object(t->data_container_len);
  const bool is_prop_edit = (t->flag & T_PROP_EDIT) != 0;

  /* Count selected elements per object and create TransData structs. */
  for (const int i : trans_data_contrainers.index_range()) {
    TransDataContainer &tc = trans_data_contrainers[i];
    Curves *curves_id = static_cast<Curves *>(tc.obedit->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();

    if (is_prop_edit) {
      tc.data_len = curves.point_num;
    }
    else {
      selection_per_object[i] = ed::curves::retrieve_selected_points(
          curves, selected_indices_per_object[i]);
      tc.data_len = selection_per_object[i].size();
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
    Curves *curves_id = static_cast<Curves *>(tc.obedit->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();

    float mtx[3][3], smtx[3][3];
    copy_m3_m4(mtx, tc.obedit->object_to_world);
    pseudoinverse_m3_m3(smtx, mtx, PSEUDOINVERSE_EPSILON);

    MutableSpan<float3> positions = curves.positions_for_write();
    if (is_prop_edit) {
      OffsetIndices<int> points_by_curve = curves.points_by_curve();
      VArray<bool> selection = curves.attributes().lookup_or_default<bool>(
          ".selection", ATTR_DOMAIN_POINT, true);
      threading::parallel_for(curves.curves_range(), 1024, [&](const IndexRange range) {
        for (const int curve_i : range) {
          bool has_any_selected = false;
          for (const int point_i : points_by_curve[curve_i]) {
            TransData *td = &tc.data[point_i];
            float *elem = reinterpret_cast<float *>(&positions[point_i]);
            copy_v3_v3(td->iloc, elem);
            copy_v3_v3(td->center, td->iloc);
            td->loc = elem;

            if (selection[point_i]) {
              has_any_selected = true;
            }

            td->flag = (selection[point_i]) ? TD_SELECTED : 0;
            td->ext = nullptr;

            copy_m3_m3(td->smtx, smtx);
            copy_m3_m3(td->mtx, mtx);
          }

          if (!has_any_selected) {
            for (const int point_i : points_by_curve[curve_i]) {
              TransData *td = &tc.data[point_i];
              td->flag |= TD_NOTCONNECTED;
            }
          }
        }
      });
    }
    else {
      IndexMask selected_indices = selection_per_object[i];
      threading::parallel_for(selected_indices.index_range(), 1024, [&](const IndexRange range) {
        for (const int selection_i : range) {
          TransData *td = &tc.data[selection_i];
          float *elem = reinterpret_cast<float *>(&positions[selected_indices[selection_i]]);
          copy_v3_v3(td->iloc, elem);
          copy_v3_v3(td->center, td->iloc);
          td->loc = elem;

          td->flag = TD_SELECTED;
          td->ext = nullptr;

          copy_m3_m3(td->smtx, smtx);
          copy_m3_m3(td->mtx, mtx);
        }
      });
    }
  }
}

static void recalcData_curves(TransInfo *t)
{
  Span<TransDataContainer> trans_data_contrainers(t->data_container, t->data_container_len);
  for (const TransDataContainer &tc : trans_data_contrainers) {
    Curves *curves_id = static_cast<Curves *>(tc.obedit->data);
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();

    curves.calculate_bezier_auto_handles();
    curves.tag_positions_changed();
    DEG_id_tag_update(&curves_id->id, ID_RECALC_GEOMETRY);
  }
}

}  // namespace blender::ed::transform::curves

/** \} */

TransConvertTypeInfo TransConvertType_Curves = {
    /*flags*/ (T_EDIT | T_POINTS),
    /*createTransData*/ blender::ed::transform::curves::createTransCurvesVerts,
    /*recalcData*/ blender::ed::transform::curves::recalcData_curves,
    /*special_aftertrans_update*/ nullptr,
};
