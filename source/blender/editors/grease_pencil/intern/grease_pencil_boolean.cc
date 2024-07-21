/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "ED_grease_pencil.hh"

#include "BKE_curves.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "BLI_polygon_clipping_2d.hh"

#include <algorithm>

namespace blender::ed::greasepencil::curveboolean {

static Set<std::string> skipped_attribute_ids(const bool keep_caps,
                                              const bool is_fill,
                                              const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return {};

    case bke::AttrDomain::Curve:
      if ((!keep_caps && !is_fill)) {
        return {
            "cyclic",
            "start_cap",
            "end_cap",
        };
      }
      else {
        return {"cyclic"};
      }
    default:
      return {};
  }
  return {};
}

void End_caps(const bool keep_first,
              const bool keep_last,
              const int dst_curve_i,
              const int src_curve_i,
              const int added_curve_num,
              const bke::AttributeAccessor &src_attributes,
              bke::MutableAttributeAccessor &dst_attributes)
{
  /* Only set the attribute if the type is not the default or if it already exists. */
  if (!(keep_first && added_curve_num == 1 && !src_attributes.contains("start_cap"))) {
    bke::SpanAttributeWriter<int8_t> dst_start_caps =
        dst_attributes.lookup_or_add_for_write_span<int8_t>("start_cap", bke::AttrDomain::Curve);

    dst_start_caps.span.drop_front(dst_curve_i).fill(GP_STROKE_CAP_TYPE_FLAT);

    if (keep_first) {
      const VArray<int> src_start_caps = *src_attributes.lookup_or_default<int>(
          "start_cap", bke::AttrDomain::Curve, GP_STROKE_CAP_TYPE_ROUND);
      dst_start_caps.span[dst_curve_i] = src_start_caps[src_curve_i];
    }

    dst_start_caps.finish();
  }

  if (!(keep_last && added_curve_num == 1 && !src_attributes.contains("end_cap"))) {
    bke::SpanAttributeWriter<int8_t> dst_end_caps =
        dst_attributes.lookup_or_add_for_write_span<int8_t>("end_cap", bke::AttrDomain::Curve);

    dst_end_caps.span.drop_front(dst_curve_i).fill(GP_STROKE_CAP_TYPE_FLAT);

    if (keep_last) {
      const VArray<int> src_end_caps = *src_attributes.lookup_or_default<int>(
          "end_cap", bke::AttrDomain::Curve, GP_STROKE_CAP_TYPE_ROUND);
      dst_end_caps.span[dst_curve_i + added_curve_num - 1] = src_end_caps[src_curve_i];
    }

    dst_end_caps.finish();
  }
}

bke::CurvesGeometry curves_geometry_cut(const bke::CurvesGeometry &src,
                                        const bke::CurvesGeometry &cut,
                                        const Span<bool> use_fill,
                                        const bool keep_caps,
                                        const Span<float2> src_pos2d,
                                        const Span<float2> cut_pos2d)
{
  if (src.points_num() < 3) {
    return bke::CurvesGeometry(src);
  }

  if (cut.points_num() < 3) {
    return bke::CurvesGeometry(src);
  }

  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();
  polygonboolean::InputMode input_mode;
  input_mode.boolean_mode = polygonboolean::BooleanMode::A_NOT_B;
  input_mode.hole_mode = polygonboolean::HoleMode::WITHOUT_HOLES;

  bke::CurvesGeometry dst = bke::CurvesGeometry();

  for (const int curve_i : src.curves_range()) {
    const IndexRange points = src_points_by_curve[curve_i];

    const Span<float2> pos_2d_a = src_pos2d.slice(points);
    const Span<float2> pos_2d_b = cut_pos2d;

    const bool is_fill = use_fill[curve_i];

    polygonboolean::BooleanResult result;
    if (is_fill) {
      result = polygonboolean::curve_boolean_calc(input_mode, pos_2d_a, pos_2d_b);
    }
    else {
      result = polygonboolean::curve_boolean_cut(pos_2d_a, pos_2d_b);
    }

    if (result.offsets.size() - 1 == 0) {
      continue;
    }

    if (!result.valid_geometry) {
      printf("Not valid geometry.\n");
    }

    const int a_size = pos_2d_a.size();
    const int b_size = pos_2d_b.size();

    const int points_num = dst.points_num();
    const int curves_num = dst.curves_num();
    const int added_curve_num = result.offsets.size() - 1;

    dst.resize(points_num + result.verts.size(), curves_num + added_curve_num);

    MutableSpan<int> offsets = dst.offsets_for_write().drop_front(curves_num);

    for (const int i : IndexRange(result.offsets.size())) {
      offsets[i] = result.offsets[i] + points_num;
    }

    dst.cyclic_for_write().drop_front(curves_num).fill(is_fill);

    const bke::AttributeAccessor src_attributes = src.attributes();
    const bke::AttributeAccessor cut_attributes = cut.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    /* End caps. */
    if (!keep_caps && !is_fill) {
      const polygonboolean::Vertex &vertex_first = result.verts.first();
      const polygonboolean::Vertex &vertex_last = result.verts.last();

      const bool keep_first = vertex_first.type == polygonboolean::VertexType::PointA &&
                              vertex_first.point_id == 0;
      const bool keep_last = vertex_last.type == polygonboolean::VertexType::PointA &&
                             vertex_last.point_id == a_size - 1;

      End_caps(keep_first,
               keep_last,
               curves_num,
               curve_i,
               added_curve_num,
               src_attributes,
               dst_attributes);
    }

    const Set<std::string> &skip = skipped_attribute_ids(
        keep_caps, is_fill, bke::AttrDomain::Curve);

    /* Copy curve attributes. */
    src_attributes.for_all(
        [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
          if (meta_data.domain != bke::AttrDomain::Curve) {
            return true;
          }
          if (skip.contains(id.name())) {
            return true;
          }
          GVArray srcR = *src_attributes.lookup(id, meta_data.domain);
          bke::GSpanAttributeWriter dstW = dst_attributes.lookup_or_add_for_write_only_span(
              id, meta_data.domain, meta_data.data_type);

          bke::attribute_math::convert_to_static_type(dstW.span.type(), [&](auto dummy) {
            using T = decltype(dummy);
            auto src_attr = srcR.typed<T>();
            auto dst_attr = dstW.span.typed<T>();

            // dst_attr.drop_back(curves_num).fill(src_attr[curve_i]);
            for (const int i : IndexRange(added_curve_num)) {
              dst_attr[curves_num + i] = src_attr[curve_i];
            }
          });
          dstW.finish();
          return true;
        });

    /* Copy/Interpolate point attributes. */
    src_attributes.for_all(
        [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
          if (meta_data.domain != bke::AttrDomain::Point) {
            return true;
          }

          GVArray src1 = *src_attributes.lookup(id, meta_data.domain);
          GVArray src2 = *cut_attributes.lookup(id, meta_data.domain);
          bke::GSpanAttributeWriter dstW = dst_attributes.lookup_or_add_for_write_only_span(
              id, meta_data.domain, meta_data.data_type);

          bke::attribute_math::convert_to_static_type(dstW.span.type(), [&](auto dummy) {
            using T = decltype(dummy);
            auto src1_attr = src1.typed<T>();
            auto src2_attr = src2.typed<T>();
            auto dst_attr = dstW.span.typed<T>();

            threading::parallel_for(
                IndexRange(result.verts.size()), 4096, [&](const IndexRange i_range) {
                  for (const int i : i_range) {
                    const polygonboolean::Vertex &vert = result.verts[i];
                    const int point_id = vert.point_id;
                    const polygonboolean::VertexType type = vert.type;
                    const int dst_point = points_num + i;

                    if (type == polygonboolean::VertexType::PointA) {
                      dst_attr[dst_point] = src1_attr[point_id + points.first()];
                    }
                    else if (type == polygonboolean::VertexType::PointB) {
                      if (src2_attr) {
                        dst_attr[dst_point] = src2_attr[point_id];
                      }
                      else {
                        dst_attr[dst_point] = src1_attr[points.first()];
                      }
                    }
                    else if (type == polygonboolean::VertexType::Intersection) {
                      const polygonboolean::IntersectionPoint &intersection =
                          result.intersections_data[point_id];

                      const T a_line = bke::attribute_math::mix2<T>(
                          intersection.alpha_a,
                          src1_attr[intersection.point_a + points.first()],
                          src1_attr[(intersection.point_a + 1) % a_size + points.first()]);

                      if (src2_attr) {
                        const T b_line = bke::attribute_math::mix2<T>(
                            intersection.alpha_b,
                            src2_attr[intersection.point_b],
                            src2_attr[(intersection.point_b + 1) % b_size]);

                        dst_attr[dst_point] = bke::attribute_math::mix2<T>(0.5, a_line, b_line);
                      }
                      else {
                        dst_attr[dst_point] = a_line;
                      }
                    }
                  }
                });
            dstW.finish();
          });

          return true;
        });
  }

  dst.update_curve_types();

  return dst;
}

}  // namespace blender::ed::greasepencil::curveboolean
