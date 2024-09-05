/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_polygon_clipping_2d.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute_math.hh"

namespace blender::bke::polygonboolean {

template<typename T>
static T interpolate_attribute_of_a_intersection_point(
    const VArray<T> src_a, const blender::polygonboolean::IntersectionPoint &inter_point)
{
  const T a0 = src_a[inter_point.point_a];
  const T a1 = src_a[(inter_point.point_a + 1) % src_a.size()];
  const float alpha_a = inter_point.alpha_a;

  return attribute_math::mix2<T>(alpha_a, a0, a1);
}

template<typename T>
static T interpolate_attribute_of_b_intersection_point(
    const VArray<T> src_b, const blender::polygonboolean::IntersectionPoint &inter_point)
{
  const T b0 = src_b[inter_point.point_b];
  const T b1 = src_b[(inter_point.point_b + 1) % src_b.size()];
  const float alpha_b = inter_point.alpha_b;

  return attribute_math::mix2<T>(alpha_b, b0, b1);
}

static Array<int2> calculate_segments(const blender::polygonboolean::BooleanResult &result)
{
  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(result.offsets);
  Array<int2> segment_indices(result.verts.size(), int2(-1, -1));

  for (const int polygon_id : points_by_polygon.index_range()) {
    const IndexRange vert_ids = points_by_polygon[polygon_id];

    int first_i = -1;
    int last_i = -1;

    for (const int i : vert_ids.index_range()) {
      const blender::polygonboolean::Vertex &vert = result.verts[vert_ids[i]];
      if (vert.type == blender::polygonboolean::VertexType::Intersection) {
        first_i = vert.point_id;
        break;
      }
    }

    for (const int i : vert_ids.index_range()) {
      const blender::polygonboolean::Vertex &vert =
          result.verts[vert_ids[vert_ids.size() - 1 - i]];
      if (vert.type == blender::polygonboolean::VertexType::Intersection) {
        last_i = vert.point_id;
        break;
      }
    }

    int inter_point = last_i;

    for (const int i : vert_ids.index_range()) {
      const int j = vert_ids[i];
      const blender::polygonboolean::Vertex &vert = result.verts[j];

      if (vert.type == blender::polygonboolean::VertexType::Intersection) {
        inter_point = vert.point_id;
      }

      segment_indices[j][0] = inter_point;
    }

    inter_point = first_i;

    for (const int i : vert_ids.index_range()) {
      const int j = vert_ids[vert_ids.size() - 1 - i];
      const blender::polygonboolean::Vertex &vert = result.verts[j];

      if (vert.type == blender::polygonboolean::VertexType::Intersection) {
        inter_point = vert.point_id;
      }

      segment_indices[j][1] = inter_point;
    }
  }

  return segment_indices;
}

template<typename T>
void interpolate_attribute_from_a_result(const VArray<T> src_a,
                                         const blender::polygonboolean::BooleanResult &result,
                                         MutableSpan<T> dst)
{
  const Array<int2> segment_indices = calculate_segments(result);

  for (const int i : result.verts.index_range()) {
    const blender::polygonboolean::Vertex &vert = result.verts[i];
    const blender::polygonboolean::VertexType &type = vert.type;

    if (type == blender::polygonboolean::VertexType::PointA) {
      dst[i] = src_a[vert.point_id];
    }
    else if (type == blender::polygonboolean::VertexType::PointB) {
      const int2 &segment = segment_indices[i];

      const blender::polygonboolean::IntersectionPoint &i0 = result.intersections_data[segment[0]];
      const blender::polygonboolean::IntersectionPoint &i1 = result.intersections_data[segment[1]];

      const T a0 = interpolate_attribute_of_a_intersection_point<T>(src_a, i0);
      const T a1 = interpolate_attribute_of_a_intersection_point<T>(src_a, i1);

      const float min_b = i0.point_b + i0.alpha_b;
      const float max_b = i1.point_b + i1.alpha_b;
      const float id_b = vert.point_id;

      const float alpha = (id_b - min_b) / (max_b - min_b);

      dst[i] = attribute_math::mix2<T>(alpha, a0, a1);
    }
    else if (type == blender::polygonboolean::VertexType::Intersection) {
      const blender::polygonboolean::IntersectionPoint &inter_point =
          result.intersections_data[vert.point_id];
      dst[i] = interpolate_attribute_of_a_intersection_point<T>(src_a, inter_point);
    }
  }
}

template<typename T>
void interpolate_attribute_from_b_result(const VArray<T> src_b,
                                         const blender::polygonboolean::BooleanResult &result,
                                         MutableSpan<T> dst)
{
  const Array<int2> segment_indices = calculate_segments(result);

  for (const int i : result.verts.index_range()) {
    const blender::polygonboolean::Vertex &vert = result.verts[i];
    const blender::polygonboolean::VertexType &type = vert.type;

    if (type == blender::polygonboolean::VertexType::PointA) {
      const int2 &segment = segment_indices[i];

      const blender::polygonboolean::IntersectionPoint &i0 = result.intersections_data[segment[0]];
      const blender::polygonboolean::IntersectionPoint &i1 = result.intersections_data[segment[1]];

      const T b0 = interpolate_attribute_of_b_intersection_point<T>(src_b, i0);
      const T b1 = interpolate_attribute_of_b_intersection_point<T>(src_b, i1);

      const float min_a = i0.point_a + i0.alpha_a;
      const float max_a = i1.point_a + i1.alpha_a;
      const float id_a = vert.point_id;

      const float alpha = (id_a - min_a) / (max_a - min_a);

      dst[i] = attribute_math::mix2<T>(alpha, b0, b1);
    }
    else if (type == blender::polygonboolean::VertexType::PointB) {
      dst[i] = src_b[vert.point_id];
    }
    else if (type == blender::polygonboolean::VertexType::Intersection) {
      const blender::polygonboolean::IntersectionPoint &inter_point =
          result.intersections_data[vert.point_id];
      dst[i] = interpolate_attribute_of_b_intersection_point<T>(src_b, inter_point);
    }
  }
}

template<typename T>
void interpolate_attribute_from_ab_result(const VArray<T> src_a,
                                          const VArray<T> src_b,
                                          const blender::polygonboolean::BooleanResult &result,
                                          MutableSpan<T> dst)
{
  for (const int i : result.verts.index_range()) {
    const blender::polygonboolean::Vertex &vert = result.verts[i];
    const blender::polygonboolean::VertexType &type = vert.type;

    if (type == blender::polygonboolean::VertexType::PointA) {
      dst[i] = src_a[vert.point_id];
    }
    else if (type == blender::polygonboolean::VertexType::PointB) {
      dst[i] = src_b[vert.point_id];
    }
    else if (type == blender::polygonboolean::VertexType::Intersection) {
      const blender::polygonboolean::IntersectionPoint &inter_point =
          result.intersections_data[vert.point_id];

      const T a = interpolate_attribute_of_a_intersection_point<T>(src_a, inter_point);
      const T b = interpolate_attribute_of_b_intersection_point<T>(src_b, inter_point);

      dst[i] = attribute_math::mix2<T>(0.5f, a, b);
    }
  }
}

}  // namespace blender::bke::polygonboolean
