/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_screen_types.h"

#include "ED_view3d.hh"

#include "GEO_join_geometries.hh"

#include "BLI_polygon_clipping_2d.hh"

#include <algorithm>

namespace blender::ed::curves::clipping {

static Set<std::string> skipped_attribute_ids(const bool keep_caps,
                                              const bool is_fill,
                                              const bool reproject,
                                              const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      if (reproject) {
        return {"position"};
      }
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

void Set_start_cap(const bool keep_first,
                   const int dst_curve_i,
                   const int src_curve_i,
                   const int added_curve_num,
                   const bke::AttributeAccessor &src_attributes,
                   bke::MutableAttributeAccessor &dst_attributes)
{
  /* Only set the attribute if the type is not the default or if it already exists. */
  if (keep_first && added_curve_num == 1 && !src_attributes.contains("start_cap")) {
    return;
  }

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

void Set_end_cap(const bool keep_last,
                 const int dst_curve_i,
                 const int src_curve_i,
                 const int added_curve_num,
                 const bke::AttributeAccessor &src_attributes,
                 bke::MutableAttributeAccessor &dst_attributes)
{
  /* Only set the attribute if the type is not the default or if it already exists. */
  if (keep_last && added_curve_num == 1 && !src_attributes.contains("end_cap")) {
    return;
  }

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

void Set_cap_attributes(const bool keep_first,
                        const bool keep_last,
                        const int dst_curve_i,
                        const int src_curve_i,
                        const int added_curve_num,
                        const bke::AttributeAccessor &src_attributes,
                        bke::MutableAttributeAccessor &dst_attributes)
{
  Set_start_cap(
      keep_first, dst_curve_i, src_curve_i, added_curve_num, src_attributes, dst_attributes);
  Set_end_cap(
      keep_last, dst_curve_i, src_curve_i, added_curve_num, src_attributes, dst_attributes);
}

static float4 transform_plane(const float4x4 &mat, const float4 &plane)
{
  float3 normal = float3(plane);
  float3 point = -normal * plane.w;

  normal = math::transform_direction(mat, normal);
  point = math::transform_point(mat, point);

  return float4(normal, -math::dot(normal, point));
}

bke::CurvesGeometry curves_geometry_cut(const bke::CurvesGeometry &src,
                                        const Span<bool> use_fill,
                                        const bool keep_caps,
                                        const ARegion &region,
                                        const float4x4 &layer_to_world,
                                        const Span<float4> normal_planes,
                                        const Span<float2> src_pos2d,
                                        const Span<float2> cut_pos2d)
{
  if (src.points_num() == 0 || cut_pos2d.size() < 3) {
    return bke::CurvesGeometry(src);
  }

  const VArray<bool> src_cyclic = src.cyclic();
  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();
  const polygonboolean::InputMode input_mode = {polygonboolean::BooleanMode::A_NOT_B,
                                                polygonboolean::HoleMode::WITHOUT_HOLES};

  Vector<bke::GeometrySet> geometry_sets;

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
      const bool is_cyclial = src_cyclic[curve_i];
      result = polygonboolean::curve_boolean_cut(is_cyclial, pos_2d_a, pos_2d_b);
    }

    if (result.offsets.size() - 1 == 0) {
      continue;
    }

    BLI_assert(result.valid_geometry);

    const int added_curve_num = result.offsets.size() - 1;
    bke::CurvesGeometry dst = bke::CurvesGeometry(result.verts.size(), added_curve_num);

    dst.offsets_for_write().copy_from(result.offsets);
    dst.cyclic_for_write().fill(is_fill);

    const bke::AttributeAccessor src_attributes = src.attributes();
    bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();

    /* End caps. */
    if (!keep_caps && !is_fill) {
      /**
       * NOTE: the last vertex in the list is not guaranteed to be the last vertex in the order
       * (i.e. the one with largest alpha)
       * But it is guaranteed to be on the latest segment, and that is all that needs to be
       * checked.
       */
      const polygonboolean::Vertex &vertex_first = result.verts.first();
      const polygonboolean::Vertex &vertex_last = result.verts.last();

      const bool keep_first = vertex_first.type == polygonboolean::VertexType::PointA &&
                              vertex_first.point_id == 0;
      const bool keep_last = vertex_last.type == polygonboolean::VertexType::PointA &&
                             vertex_last.point_id == pos_2d_a.size() - 1;

      Set_cap_attributes(
          keep_first, keep_last, 0, curve_i, added_curve_num, src_attributes, dst_attributes);
    }

    const bool reproject = result.intersections_data.size() != 0;

    const Set<std::string> &curve_skip = skipped_attribute_ids(
        keep_caps, is_fill, reproject, bke::AttrDomain::Curve);

    /* Copy curve attributes. */
    src_attributes.for_all(
        [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
          if (meta_data.domain != bke::AttrDomain::Curve) {
            return true;
          }
          if (curve_skip.contains(id.name())) {
            return true;
          }
          GVArray srcR = *src_attributes.lookup(id, meta_data.domain);
          bke::GSpanAttributeWriter dstW = dst_attributes.lookup_or_add_for_write_only_span(
              id, meta_data.domain, meta_data.data_type);

          bke::attribute_math::convert_to_static_type(dstW.span.type(), [&](auto dummy) {
            using T = decltype(dummy);
            auto src_attr = srcR.typed<T>();
            auto dst_attr = dstW.span.typed<T>();

            // dst_attr.fill(src_attr[curve_i]); /* TODO */
            for (const int i : IndexRange(added_curve_num)) {
              dst_attr[i] = src_attr[curve_i];
            }
          });
          dstW.finish();
          return true;
        });

    const Set<std::string> &point_skip = skipped_attribute_ids(
        keep_caps, is_fill, reproject, bke::AttrDomain::Point);

    /* Copy/Interpolate point attributes. */
    src_attributes.for_all(
        [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
          if (meta_data.domain != bke::AttrDomain::Point) {
            return true;
          }
          if (point_skip.contains(id.name())) {
            return true;
          }

          GVArray src1 = (*src_attributes.lookup(id, meta_data.domain)).slice(points);
          bke::GSpanAttributeWriter dstW = dst_attributes.lookup_or_add_for_write_only_span(
              id, meta_data.domain, meta_data.data_type);

          bke::attribute_math::convert_to_static_type(dstW.span.type(), [&](auto dummy) {
            using T = decltype(dummy);
            VArray<T> src1_attr = src1.typed<T>();
            MutableSpan<T> dst_attr = (dstW.span.typed<T>());

            polygonboolean::interpolate_data_from_a_result<T>(src1_attr, result, dst_attr);

            dstW.finish();
          });

          return true;
        });

    if (reproject) {
      MutableSpan<float3> positions = dst.positions_for_write();
      const Array<float2> pos2d = polygonboolean::interpolate_data_from_ab_result<float2>(
          pos_2d_a, pos_2d_b, result);

      const float4 &plane = transform_plane(layer_to_world, normal_planes[curve_i]);
      const float4x4 world_to_layer = math::invert(layer_to_world);
      for (const int i : pos2d.index_range()) {
        ED_view3d_win_to_3d_on_plane(&region, plane, pos2d[i], false, positions[i]);
        positions[i] = math::transform_point(world_to_layer, positions[i]);
      }
    }

    Curves *target_id = curves_new_nomain(std::move(dst));
    geometry_sets.append(bke::GeometrySet::from_curves(target_id));
  }

  if (geometry_sets.is_empty()) {
    return bke::CurvesGeometry();
  }

  bke::GeometrySet joined_curves = geometry::join_geometries(geometry_sets, {});
  bke::CurvesGeometry out_dst = joined_curves.get_curves_for_write()->geometry.wrap();

  out_dst.update_curve_types();

  return out_dst;
}

}  // namespace blender::ed::curves::clipping
