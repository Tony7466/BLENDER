/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

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

  void on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension) override;
  void on_stroke_done(const bContext &C) override;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct EraseOperationExecutor {

  EraseOperationExecutor(const bContext & /*C*/) {}

  void interpolate_point(blender::bke::CurvesGeometry &dst,
                         const int dst_index,
                         const blender::bke::CurvesGeometry &src,
                         const int src_index_0,
                         const int src_index_1,
                         const float factor,
                         const blender::bke::AnonymousAttributePropagationInfo &propagation_info)
  {
    using namespace blender;
    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    {
      attribute.dst.span.slice(IndexRange(dst_index, 1))
          .copy_from(attribute.src.slice(IndexRange(src_index_0, 1)));
      attribute.dst.finish();
    }
  }

  void copy_point(blender::bke::CurvesGeometry &dst,
                  const int dst_index,
                  const blender::bke::CurvesGeometry &src,
                  const int src_index,
                  const blender::bke::AnonymousAttributePropagationInfo &propagation_info)
  {
    using namespace blender;
    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    {
      attribute.dst.span.slice(IndexRange(dst_index, 1))
          .copy_from(attribute.src.slice(IndexRange(src_index, 1)));
      attribute.dst.finish();
    }
  }

  void copy_point(blender::bke::CurvesGeometry &dst,
                  const IndexRange dst_range,
                  const blender::bke::CurvesGeometry &src,
                  const IndexRange src_range,
                  const blender::bke::AnonymousAttributePropagationInfo &propagation_info)
  {
    using namespace blender;
    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    {
      attribute.dst.span.slice(dst_range).copy_from(attribute.src.slice(src_range));
      attribute.dst.finish();
    }
  }

  template<typename T>
  static inline void interpolation(const T &a, const T &b, Span<float> factors, MutableSpan<T> dst)
  {
    for (const int i : dst.index_range()) {
      dst[i] = bke::attribute_math::mix2<T>(factors[i], a, b);
    }
  }

  void set_curves_range(CurvesGeometry &dst, const int curve_index, const int first_pt_index) {}

  void execute(EraseOperation & /*self*/,
               const bContext &C,
               const StrokeExtension &stroke_extension)
  {
    using namespace blender::bke;
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    ARegion *region = CTX_wm_region(&C);
    Object *obact = CTX_data_active_object(&C);
    Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

    const int MAX_NB_CUTS = 3;

    /* Get the tool's data */
    float2 mouse_position = stroke_extension.mouse_position;
    float eraser_radius = stroke_extension.pressure * 100;  // TODO : Fix the computation of radius

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
    const offset_indices::OffsetIndices<int> src_points_by_curve = src.points_by_curve();
    const VArray<bool> cyclic = src.cyclic();

    int points_to_add = 0;
    int points_to_remove = 0;
    int curve_count = 0;
    Array<int> cuts_count(src.points_num() + src.curves_num(), 0);
    Array<float> segment_intersections(MAX_NB_CUTS * src.points_range().size(), -1.0f);
    Array<bool> keep_point(src.points_num(), true);

    /* Compute intersection points with the eraser */
    // threading::parallel_for(curves.curves_range(), 256, [&](const IndexRange &range) {
    //   for (const int curve_i : range) {

    for (const int curve_i : src.curves_range()) {
      IndexRange points = src_points_by_curve[curve_i];
      const bool is_cyclic = cyclic[curve_i];
      const int nb_pts = points.size();
      const int first_pt = points.first();
      bool first_point_trimmed = false;

      // threading::parallel_for(points.drop_back(1), 256, [&](const IndexRange range) {
      //   for (const int segment_i : range) {

      for (const int segment_i : points.drop_back(1)) {
        const float3 pos = deformation.positions[segment_i];
        int intersection_index = MAX_NB_CUTS * segment_i;

        /* Compute screen space point position */
        float2 pos_view{};
        ED_view3d_project_float_global(region, pos, pos_view, V3D_PROJ_TEST_NOP);

        float3 pos_after = deformation.positions[segment_i + 1];
        float2 pos_after_view{};
        ED_view3d_project_float_global(region, pos, pos_view, V3D_PROJ_TEST_NOP);

        /* Check if the point is inside the eraser */
        if (len_squared_v2v2(pos_view, mouse_position) <= eraser_radius * eraser_radius) {
          points_to_remove++;
          keep_point[segment_i] = false;

          if (segment_i == points.first()) {
            first_point_trimmed = true;
          }
          segment_intersections[intersection_index] = -1.0;
          intersection_index++;
        }
        else {
          cuts_count[segment_i + curve_i]++;
        }

        /* Compute intersections between the current segment and the eraser's area */
        float2 inter0{};
        float2 inter1{};
        const int nb_inter = isect_line_sphere_v2(
            pos_view, pos_after_view, mouse_position, eraser_radius, inter0, inter1);

        /* The function above returns the intersections with the (infinite) line,
         * so we have to make sure they lie within the segment.  */
        float mu0 = (nb_inter > 0) ?
                        (len_v2(inter0 - pos_view) / len_v2(pos_after_view - pos_view)) :
                        -1.0;
        float mu1 = (nb_inter > 1) ?
                        (len_v2(inter1 - pos_view) / len_v2(pos_after_view - pos_view)) :
                        -1.0;

        if (IN_RANGE(mu0, 0, 1)) {
          cuts_count[segment_i + curve_i]++;
          segment_intersections[intersection_index] = mu0;
          intersection_index++;
          points_to_add++;
        }
        if (IN_RANGE(mu1, 0, 1)) {
          cuts_count[segment_i + curve_i]++;
          segment_intersections[intersection_index] = mu1;
          points_to_add++;
        }

        if (segment_i + 1 == points.last()) {
          /* Last point of the curve */
          if (len_squared_v2v2(pos_after_view, mouse_position) <= eraser_radius * eraser_radius) {
            points_to_remove++;
            keep_point[segment_i + 1] = false;
          }
          else {
            cuts_count[segment_i + curve_i + 1]++;
            segment_intersections[MAX_NB_CUTS * (segment_i + 1)] = 0.0;
          }
        }
      }
      // });

      if (is_cyclic) {
        /* TODO */
      }
    }
    // });

    /* Compute the accumulated offset */
    OffsetIndices<int> acc_offsets = blender::offset_indices::accumulate_counts_to_offsets(
        cuts_count);

    std::cout << "acc_offsets: " << std::endl;
    for (const int i : acc_offsets.index_range()) {
      std::cout << acc_offsets[i] << ", ";
    }
    std::cout << std::endl;

    /* Create the new curves geometry*/
    int point_count = src.points_num() - points_to_remove + points_to_add;
    CurvesGeometry dst = bke::curves::copy_only_curve_domain(src);
    MutableSpan<int> dst_curve_offsets = dst.offsets_for_write();

    /* Fill the curves offsets
       TODO : For now, we do not account for cuts. */
    IndexMask src_curves_range(src.curves_range());
    src_curves_range.foreach_index([&](const int curve_index) {
      const int curve_first_point = src_points_by_curve[curve_index].first();
      dst_curve_offsets[curve_index] = acc_offsets[curve_first_point + curve_index].first();
    });
    dst_curve_offsets.last() = acc_offsets.total_size();

    /* Resize point data to account for new points */
    dst.resize(point_count, dst.curves_num());

    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    OffsetIndices<int> dst_points_by_curve = dst.points_by_curve();
    AnonymousAttributePropagationInfo propagation_info{};

    /* Copy/Interpolate point data */
    for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
             src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    {
      bke::attribute_math::convert_to_static_type(attribute.dst.span.type(), [&](auto dummy) {
        using T = decltype(dummy);

        src_curves_range.foreach_index(GrainSize(512), [&](const int curve_i) {
          /* Copy/Interpolate point data of a single curve */
          const IndexRange src_points = src_points_by_curve[curve_i];
          const IndexRange src_segments = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                     curve_i);

          OffsetIndices<int> dst_offset = acc_offsets.slice(src_segments);
          const IndexRange dst_points = dst_points_by_curve[curve_i];

          const Span<T> src_attr = attribute.src.typed<T>().slice(src_points);
          MutableSpan<T> dst_attr = attribute.dst.span.typed<T>().slice(dst_points);

          threading::parallel_for(
              src_attr.index_range().drop_back(1), 1024, [&](IndexRange range) {
                for (const int src_point_index : range) {
                  const IndexRange segment_points = dst_offset[src_point_index];
                  if (segment_points.size() == 0) {
                    continue;
                  }

                  Span<float> factors = segment_intersections.as_span().slice(
                      IndexRange(3 * src_point_index, segment_points.size()));

                  // dst_factors.slice(segment_points)

                  interpolation<T>(src_attr[src_point_index],
                                   src_attr[src_point_index + 1],
                                   factors,
                                   dst_attr.slice(segment_points));
                }
              });
          attribute.dst.finish();
        });
      });
    }

    // /* Copy point domain. */
    // AnonymousAttributePropagationInfo propagation_info{};

    // const bke::AttributeAccessor src_attributes = src.attributes();
    // bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    // const OffsetIndices<int> src_points_by_curve = src.points_by_curve();
    // const OffsetIndices<int> dst_points_by_curve = dst.points_by_curve();

    // for (auto &attribute : bke::retrieve_attributes_for_transfer(
    //          src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
    // {
    //   bke::curves::copy_point_data(src_points_by_curve,
    //                                dst_points_by_curve,
    //                                dst.curves_range(),
    //                                attribute.src,
    //                                attribute.dst.span);
    //   attribute.dst.finish();
    // }

    drawing.geometry.wrap() = dst;
    drawing.tag_positions_changed();
  }
};

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