/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_task.hh"

#include "BKE_context.h"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "ED_view3d.h"

#include "WM_api.h"
#include "WM_types.h"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class EraseOperation : public GreasePencilStrokeOperation {

 public:
  ~EraseOperation() override {}

  void on_stroke_begin(const bContext &C, const StrokeExtension &stroke_extension) /*override*/;
  void on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension) override;
  void on_stroke_done(const bContext &C) override;

  bool set_flat_caps = true;
  float radius = 50;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct EraseOperationExecutor {

  EraseOperationExecutor(const bContext & /*C*/) {}

  void execute(EraseOperation &self, const bContext &C, const StrokeExtension &stroke_extension)
  {
    using namespace blender::bke;
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    ARegion *region = CTX_wm_region(&C);
    Object *obact = CTX_data_active_object(&C);
    Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

    /* Get the tool's data */
    float2 mouse_position = stroke_extension.mouse_position;
    float eraser_radius = stroke_extension.pressure * self.radius;

    /* Get the grease pencil drawing */
    GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);
    const int drawing_index = grease_pencil.get_active_layer()->drawing_index_at(scene->r.cfra);
    bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
            ob_eval, *obact, drawing_index);

    blender::bke::greasepencil::Drawing &drawing =
        *reinterpret_cast<blender::bke::greasepencil::Drawing *>(
            grease_pencil.drawings_for_write()[drawing_index]);

    CurvesGeometry &src = drawing.strokes_for_write();
    const VArray<bool> src_cyclic = src.cyclic();
    const Array<int> src_points_to_curve = src.point_to_curve_map();
    const int src_points_num = src.points_num();
    const int src_curves_num = src.curves_num();
    offset_indices::OffsetIndices<int> src_points_by_curves = src.points_by_curve();

    /* Compute screen space positions */
    Array<float2> screen_space_positions(src_points_num);
    for (int point_index = 0; point_index < src_points_num; point_index++) {
      ED_view3d_project_float_global(region,
                                     deformation.positions[point_index],
                                     screen_space_positions[point_index],
                                     V3D_PROJ_TEST_NOP);
    }

    auto compute_intersection_parameter = [](float2 p0, float2 p1, float2 inter) {
      float mu = (len_v2(inter - p0) / len_v2(p1 - p0));
      float sign_mu = (dot_v2v2(inter - p0, p1 - p0) < 0) ? -1.0 : 1.0;
      return sign_mu * mu;
    };

    /* Check segments that have an intersection */
    Array<bool> has_intersection(src_points_num, false);
    Array<int> nb_intersections(src_points_num, 0);
    int intersection_count = 0;
    for (int curve_index = 0; curve_index < src_curves_num; curve_index++) {
      IndexRange src_point_range = src_points_by_curves[curve_index];

      for (int src_point_index : src_point_range.drop_back(1)) {
        const float2 pos_view = screen_space_positions[src_point_index];
        const float2 pos_after_view = screen_space_positions[src_point_index + 1];

        /* Compute intersections between the current segment and the eraser's area */
        float2 inter0{};
        float2 inter1{};
        const int nb_inter = isect_line_sphere_v2(
            pos_view, pos_after_view, mouse_position, eraser_radius, inter0, inter1);

        /* The function above returns the intersections with the (infinite) line,
         * so we have to make sure they lie within the segment.  */
        float mu0 = (nb_inter > 0) ?
                        compute_intersection_parameter(pos_view, pos_after_view, inter0) :
                        -1.0;
        float mu1 = (nb_inter > 1) ?
                        compute_intersection_parameter(pos_view, pos_after_view, inter1) :
                        -1.0;

        has_intersection[src_point_index] = IN_RANGE(mu0, 0, 1) || IN_RANGE(mu1, 0, 1);
        nb_intersections[src_point_index] = int(IN_RANGE(mu0, 0, 1)) + int(IN_RANGE(mu1, 0, 1));
        intersection_count += nb_intersections[src_point_index];
      }
      if (src_cyclic[curve_index]) {
        const int src_last_point = src_point_range.last();
        const float2 pos_view = screen_space_positions[src_last_point];
        const float2 pos_after_view = screen_space_positions[src_point_range.first()];

        /* Compute intersections between the current segment and the eraser's area */
        float2 inter0{};
        float2 inter1{};
        const int nb_inter = isect_line_sphere_v2(
            pos_view, pos_after_view, mouse_position, eraser_radius, inter0, inter1);

        /* The function above returns the intersections with the (infinite) line,
         * so we have to make sure they lie within the segment.  */
        float mu0 = (nb_inter > 0) ?
                        compute_intersection_parameter(pos_view, pos_after_view, inter0) :
                        -1.0;
        float mu1 = (nb_inter > 1) ?
                        compute_intersection_parameter(pos_view, pos_after_view, inter1) :
                        -1.0;

        has_intersection[src_last_point] = IN_RANGE(mu0, 0, 1) || IN_RANGE(mu1, 0, 1);
        nb_intersections[src_last_point] = int(IN_RANGE(mu0, 0, 1)) + int(IN_RANGE(mu1, 0, 1));
        intersection_count += nb_intersections[src_last_point];
      }
    }

    /* Check if points are inside the eraser */
    Array<bool> is_point_inside(src_points_num, false);
    int point_inside_count = 0;
    for (int point_index = 0; point_index < src_points_num; point_index++) {
      const float2 pos_view = screen_space_positions[point_index];
      is_point_inside[point_index] = (len_squared_v2v2(pos_view, mouse_position) <=
                                      eraser_radius * eraser_radius);
      point_inside_count += int(is_point_inside[point_index]);
    }

    /* Compute the factors of the destination points */
    const int dst_points_num = src_points_num + intersection_count - point_inside_count;
    Array<float> dst_points_parameters(dst_points_num);
    Array<bool> is_cut(dst_points_num, false);
    Array<int> src_pivot_point(src_curves_num, -1);
    Array<int> dst_interm_curves_offsets(src_curves_num + 1, 0);
    int dst_point_index = -1;
    for (int src_curve_index = 0; src_curve_index < src_curves_num; src_curve_index++) {
      IndexRange src_point_range = src_points_by_curves[src_curve_index];
      const int src_point_last = src_point_range.last();

      for (int src_point_index : src_point_range) {
        if (!is_point_inside[src_point_index]) {
          dst_points_parameters[++dst_point_index] = float(src_point_index);
        }
        if (has_intersection[src_point_index]) {
          const float2 pos_view = screen_space_positions[src_point_index];
          const int src_next_point_index = (src_point_index != src_point_last) ?
                                               (src_point_index + 1) :
                                               (src_point_range.first());
          const float2 pos_after_view = screen_space_positions[src_next_point_index];

          /* Compute intersections between the current segment and the eraser's area */
          float2 inter0{};
          float2 inter1{};
          const int nb_inter = isect_line_sphere_v2(
              pos_view, pos_after_view, mouse_position, eraser_radius, inter0, inter1);

          /* The function above returns the intersections with the (infinite) line,
           * so we have to make sure they lie within the segment.  */
          float mu0 = (nb_inter > 0) ?
                          compute_intersection_parameter(pos_view, pos_after_view, inter0) :
                          -1.0;
          float mu1 = (nb_inter > 1) ?
                          compute_intersection_parameter(pos_view, pos_after_view, inter1) :
                          -1.0;

          if (mu0 > mu1) {
            std::swap(mu0, mu1);
          }

          if (IN_RANGE(mu0, 0, 1)) {
            dst_points_parameters[++dst_point_index] = src_point_index + mu0;
            is_cut[dst_point_index] = true;
          }
          if (IN_RANGE(mu1, 0, 1)) {
            dst_points_parameters[++dst_point_index] = src_point_index + mu1;
            is_cut[dst_point_index] = true;
          }

          if (src_cyclic[src_curve_index] &&
              (is_point_inside[src_point_index] || (nb_intersections[src_point_index] == 2)))
          {
            src_pivot_point[src_curve_index] = dst_point_index;
          }
        }
      }
      dst_interm_curves_offsets[src_curve_index + 1] = dst_point_index + 1;
    }

    /* Shift the indices for cyclic curves */
    Array<bool> src_now_cyclic(src_curves_num);
    for (int src_curve_index : src.curves_range()) {
      const int pivot_point = src_pivot_point[src_curve_index];

      if (pivot_point == -1) {
        /* Either the curve was not cyclic or it wasn't cut : no need to change it */
        src_now_cyclic[src_curve_index] = src_cyclic[src_curve_index];
        continue;
      }

      /* The cyclic curve was cut, so this curve is not cyclic anymore */
      src_now_cyclic[src_curve_index] = false;

      /* The cyclic curve was cut
       * and we have to shift points so that we keep the cyclic segment */
      const int dst_interm_first = dst_interm_curves_offsets[src_curve_index];
      const int dst_interm_last = dst_interm_curves_offsets[src_curve_index + 1];
      std::rotate(dst_points_parameters.begin() + dst_interm_first,
                  dst_points_parameters.begin() + pivot_point,
                  dst_points_parameters.begin() + dst_interm_last);
      std::rotate(is_cut.begin() + dst_interm_first,
                  is_cut.begin() + pivot_point,
                  is_cut.begin() + dst_interm_last);
    }

    /* Compute the destination curve offsets*/
    Vector<int> dst_curves_offset;
    Vector<int> dst_to_src_curve_index;
    int dst_offset = 0;
    dst_curves_offset.append(0);
    for (int src_curve_index : IndexRange(src_curves_num)) {
      IndexRange dst_point_range(dst_interm_curves_offsets[src_curve_index],
                                 dst_interm_curves_offsets[src_curve_index + 1] -
                                     dst_interm_curves_offsets[src_curve_index]);
      int length_of_current = 0;

      for (int dst_point_index : dst_point_range) {
        const int src_point_index = std::floor(dst_points_parameters[dst_point_index]);
        if ((length_of_current > 0) && is_cut[dst_point_index] && is_point_inside[src_point_index])
        {
          /* This is the new first point of a curve */
          dst_curves_offset.append(dst_point_index);
          dst_to_src_curve_index.append(src_curve_index);
          length_of_current = 0;
        }
        ++length_of_current;
      }

      if (length_of_current != 0) {
        /* End of a source curve : add a new curve */
        dst_offset += length_of_current;
        dst_curves_offset.append(dst_point_range.one_after_last());
        dst_to_src_curve_index.append(src_curve_index);
      }
    }
    const int dst_curves_num = dst_curves_offset.size() - 1;

    /* Create the new curves geometry*/
    CurvesGeometry dst(dst_points_num, dst_curves_num);
    MutableSpan<int> dst_offsets_for_write = dst.offsets_for_write();
    for (int dst_curve_index : IndexRange(dst_curves_num)) {
      dst_offsets_for_write[dst_curve_index] = dst_curves_offset[dst_curve_index];
    }

    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();
    AnonymousAttributePropagationInfo propagation_info{};

    /* Copy curves data */
    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_CURVE, propagation_info, {"cyclic"}))
    {
      bke::attribute_math::convert_to_static_type(attribute.dst.span.type(), [&](auto dummy) {
        using T = decltype(dummy);
        auto src_attr = attribute.src.typed<T>();
        auto dst_attr = attribute.dst.span.typed<T>();

        for (int dst_curve_index : IndexRange(dst_curves_num)) {
          const int src_curve_index = dst_to_src_curve_index[dst_curve_index];
          dst_attr[dst_curve_index] = src_attr[src_curve_index];
        }
        attribute.dst.finish();
      });
    }

    /* Update the cyclic attribute */
    MutableSpan<bool> dst_cyclic = dst.cyclic_for_write();
    for (int dst_curve_index : dst.curves_range()) {
      const int src_curve_index = dst_to_src_curve_index[dst_curve_index];
      dst_cyclic[dst_curve_index] = src_now_cyclic[src_curve_index];
    }

    /* Display intersections with flat caps */
    if (self.set_flat_caps) {
      SpanAttributeWriter<int8_t> dst_start_caps =
          dst.attributes_for_write().lookup_or_add_for_write_span<int8_t>("start_cap",
                                                                          ATTR_DOMAIN_CURVE);
      SpanAttributeWriter<int8_t> dst_end_caps =
          dst.attributes_for_write().lookup_or_add_for_write_span<int8_t>("end_cap",
                                                                          ATTR_DOMAIN_CURVE);
      offset_indices::OffsetIndices<int> dst_points_by_curve = dst.points_by_curve();
      for (int dst_curve_index : dst.curves_range()) {
        IndexRange dst_curve_range = dst_points_by_curve[dst_curve_index];
        if (is_cut[dst_curve_range.first()]) {
          dst_start_caps.span[dst_curve_index] = GP_STROKE_CAP_TYPE_FLAT;
        }
        if (is_cut[dst_curve_range.last()]) {
          dst_end_caps.span[dst_curve_index] = GP_STROKE_CAP_TYPE_FLAT;
        }
      }
      dst_start_caps.finish();
      dst_end_caps.finish();
    }

    /* Copy/Interpolate point data */
    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    {
      bke::attribute_math::convert_to_static_type(attribute.dst.span.type(), [&](auto dummy) {
        using T = decltype(dummy);
        auto src_attr = attribute.src.typed<T>();
        auto dst_attr = attribute.dst.span.typed<T>();

        for (int dst_point_index = 0; dst_point_index < dst_points_num; ++dst_point_index) {
          const float dst_param = dst_points_parameters[dst_point_index];
          const int src_point_index = std::floor(dst_param);

          if (is_cut[dst_point_index]) {
            const float src_pt_factor = dst_param - src_point_index;

            const int src_curve_index = src_points_to_curve[src_point_index];
            const IndexRange src_curve_range = src_points_by_curves[src_curve_index];
            const int src_next_point_index = (src_point_index == src_curve_range.last()) ?
                                                 src_curve_range.first() :
                                                 (src_point_index + 1);

            dst_attr[dst_point_index] = bke::attribute_math::mix2<T>(
                src_pt_factor, src_attr[src_point_index], src_attr[src_next_point_index]);
          }
          else {
            dst_attr[dst_point_index] = src_attr[src_point_index];
          }
        }
        attribute.dst.finish();
      });
    }

    drawing.geometry.wrap() = dst;
    drawing.tag_positions_changed();

    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
  }
};

void EraseOperation::on_stroke_begin(const bContext & /*C*/,
                                     const StrokeExtension & /*stroke_extension*/)
{
  // TODO
  // Get the tools settings, and set the member attribute accordingly
}

void EraseOperation::on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension)
{
  EraseOperationExecutor executor{C};
  executor.execute(*this, C, stroke_extension);
}

void EraseOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_erase_operation()
{
  return std::make_unique<EraseOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil