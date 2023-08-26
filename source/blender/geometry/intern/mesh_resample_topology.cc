/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_linear_allocator.hh"
#include "BLI_map.hh"
#include "BLI_offset_indices.hh"
#include "BLI_string_ref.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.h"

#include "GEO_builder.hh"
#include "GEO_mesh_resample_topology.hh"

namespace blender::geometry {

class ResampledPair {
 public:
  bool flip_axis;

  /*
   * A     -     -     B
   * |\  / | \ / | \ / | <-
   * C - - - - - - - - D
   * total_connectios: Is number of edge-lines for side pair without bounding pair.
   */
  int total_connectios;
  int less_points;

  int t_max_edges;
  int t_min_edges;

  /*
   * A     -     -     B
   * |\  / | \ / | \ / |
   * C - - - - - - - - D
   *   ^             ^
   *   |             |
   * corner_points: Is points of side corner.
   */
  int corner_points;

  /*
   * A     -     -     B
   * |\  / | \ / | \ / |
   * C - - - - - - - - D
   *     ^ ^ ^ ^ ^ ^
   *     | | | | | |
   * body_points: Is points of side body.
   */
  int body_points;

  ResampledPair(const int &total_points_a, const int &total_points_b)
  {
    flip_axis = total_points_a >= total_points_b;

    total_connectios = math::max<int>(total_points_a, total_points_b);
    less_points = math::min<int>(total_points_a, total_points_b);

    t_max_edges = total_connectios + 1;
    t_min_edges = less_points + 1;

    corner_points = this->corner_for_size(t_max_edges, t_min_edges);

    body_points = this->body_of_side_without_corners(total_connectios, corner_points);
  }

 protected:
  int corner_for_size(const int &edge_max, const int &edge_min) const
  {
    return edge_max / (edge_min * 2);
  }

  int body_of_side_without_corners(const int &side, const int &corner) const
  {
    return side - corner * 2;
  }
};

class ResampleQuad {
 public:
  const ResampledPair horizontal;
  const ResampledPair vertical;

 public:
  ResampleQuad(const int &left_added_points,
               const int &right_added_points,
               const int &top_added_points,
               const int &bottom_added_points)
      : horizontal(left_added_points, right_added_points),
        vertical(top_added_points, bottom_added_points)
  {
  }

  int total_points() const
  {
    const int body_body_points = horizontal.body_points * vertical.body_points;

    const int body_corner_points_hor = horizontal.body_points * vertical.corner_points;
    const int body_corner_points_ver = vertical.body_points * horizontal.corner_points;

    const int side_corner_points_hor = horizontal.total_connectios * vertical.corner_points;
    const int side_corner_points_ver = vertical.total_connectios * horizontal.corner_points;

    const int corner_corner_duply_sub = horizontal.corner_points * vertical.corner_points;

    const int total_points = body_body_points + body_corner_points_hor + body_corner_points_ver +
                             side_corner_points_hor + side_corner_points_ver -
                             corner_corner_duply_sub;

    BLI_assert(total_points >= 0);
    return vertical.total_connectios * horizontal.total_connectios;
  }
};

namespace propagation {

template<typename T>
void edges_have_created_new_verts(const OffsetIndices<int> offsets,
                                  const Span<int2> edges,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{
  for (const int index : offsets.index_range()) {
    const int2 edge = edges[index];

    const T value_on_a_edge_vertex = src[edge[0]];
    const T value_on_b_edge_vertex = src[edge[1]];

    const IndexRange range = offsets[index];
    const int size = range.size();

    for (const int index_in_edge : IndexRange(size)) {
      const float factor = float(index_in_edge + 1) / float(size + 1);
      dst[range.start() + index_in_edge] = bke::attribute_math::mix2(
          factor, value_on_a_edge_vertex, value_on_b_edge_vertex);
    }
  }
}

template<typename T>
void edges_have_created_new_edges(const OffsetIndices<int> offsets,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{
  for (const int index : offsets.index_range()) {
    const IndexRange new_edges = offsets[index];
    const T edge_value = src[index];
    dst.slice(new_edges).fill(edge_value);
  }
}

template<typename T>
void polys_have_created_new_verts(const OffsetIndices<int> /*offsets*/,
                                  const OffsetIndices<int> /*poly_offsets*/,
                                  const Span<int> /*src_corner_verts*/,
                                  const Span<int> /*src_corner_edges*/,
                                  const Span<int> /*resample_edge_num*/,
                                  const Span<T> /*src*/,
                                  MutableSpan<T> /*dst*/)
{
  /*
    Array<int> vertical_linkeds;
    Array<int> horizontal_linkeds;

    for (const int poly_i : polys.index_range()) {
      const MPoly src_poly = polys[poly_i];
      const Span<MLoop> src_loops = loops.slice(src_poly.loopstart, src_poly.totloop);
      const IndexRange poly_vertices = offsets[poly_i];

      if (src_loops.size() != 4) {
        return;
      }

      const int &points_a = src_loops[0].v;
      const int &points_b = src_loops[1].v;
      const int &points_c = src_loops[2].v;
      const int &points_d = src_loops[3].v;

      const int &points_num_edge_a = resample_edge_num[src_loops[0].e];
      const int &points_num_edge_b = resample_edge_num[src_loops[1].e];
      const int &points_num_edge_c = resample_edge_num[src_loops[2].e];
      const int &points_num_edge_d = resample_edge_num[src_loops[3].e];

      const ResampleQuad resample(
          points_num_edge_a, points_num_edge_c, points_num_edge_b, points_num_edge_d);

      const int &hor_connection = resample.horizontal.total_connectios;
      const int &hor_other_side = resample.horizontal.less_points;
      const int &ver_connection = resample.vertical.total_connectios;
      const int &ver_other_side = resample.vertical.less_points;

      horizontal_linkeds.reinitialize(hor_connection);
      vertical_linkeds.reinitialize(ver_connection);

      for (const int index : IndexRange(hor_connection)) {
        const float d_index = float((hor_other_side + 1) * (1 + index));
        const float d_total = float(hor_connection + 1);
        horizontal_linkeds[index] = int(std::round(d_index / d_total));
      }

      for (const int index : IndexRange(ver_connection)) {
        const float d_index = float((ver_other_side + 1) * (1 + index));
        const float d_total = float(ver_connection + 1);
        vertical_linkeds[index] = int(std::round(d_index / d_total));
      }

      const auto intersection =
          [](const float2 p1, const float2 p2, const float2 p3, const float2 p4) -> float2 {
        float2 result = {};
        isect_seg_seg_v2_point_ex(p1, p2, p3, p4, 0.2f, result);
        /*
        printf(">\n");
        printf(" - x1: %f, y1: %f;\n", p1.x, p1.y);
        printf(" - x2: %f, y2: %f;\n", p2.x, p2.y);
        printf(" - x3: %f, y3: %f;\n", p3.x, p3.y);
        printf(" - x4: %f, y4: %f;\n", p4.x, p4.y);
        printf(" - xr: %f, yr: %f;\n", result.x, result.y);
        *//*
      return result;
    };

    const auto colculate_result = [&](const int &index_h, const int &index_v) -> float2 {
      const float2 uv_point_a = {0.0f, float(index_h + 1) / (hor_connection + 1)};
      const float2 uv_point_b = {1.0f, float(horizontal_linkeds[index_h]) / (hor_other_side + 1)};
      const float2 uv_point_c = {float(index_v + 1) / (ver_connection + 1), 0.0f};
      const float2 uv_point_d = {float(vertical_linkeds[index_v]) / (ver_other_side + 1), 1.0f};

      const float2 uv_point = intersection(uv_point_a, uv_point_b, uv_point_c, uv_point_d);

      return uv_point;
    };

    int point_index = 0;

    const IndexRange f_h(resample.horizontal.total_connectios);
    const IndexRange f_v(resample.vertical.total_connectios);

    for (const int index_h : f_h) {
      for (const int index_v : f_v) {
        const float2 uv_point = colculate_result(index_h, index_v);

        dst[point_index] = bke::attribute_math::mix2(
            uv_point.x,
            bke::attribute_math::mix2(uv_point.y, src[points_a], src[points_b]),
            bke::attribute_math::mix2(uv_point.y, src[points_d], src[points_c]));

        point_index++;
      }
    }
    continue;
    const IndexRange l_h(resample.horizontal.corner_points,
                         resample.horizontal.body_points + resample.horizontal.corner_points);
    const IndexRange l_v(resample.vertical.total_connectios);

    for (const int index_h : l_h) {
      for (const int index_v : l_v) {
        const float2 uv_point = colculate_result(index_h, index_v);

        dst[point_index] = bke::attribute_math::mix2(
            uv_point.x,
            bke::attribute_math::mix2(uv_point.y, src[points_a], src[points_b]),
            bke::attribute_math::mix2(uv_point.y, src[points_d], src[points_c]));

        point_index++;
      }
    }
  }*/
}

template<typename T> void polys_have_created_new_edges() {}

template<typename T>
void loops_have_created_new_loops(const OffsetIndices<int> offsets,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{
  for (const int index : offsets.index_range()) {
    const IndexRange new_edges = offsets[index];
    const T edge_value = src[index];
    dst.slice(new_edges).fill(edge_value);
  }
}

template<typename T> void polys_have_created_new_polys(const Span<T> src, MutableSpan<T> dst)
{
  dst.copy_from(src);
}

template<typename T> void polys_have_created_new_loops() {}

void attribute_on_domain(const eAttrDomain domain,
                         const builder::MeshBuilder &builder,
                         const Span<int> resample_edge_num,
                         const bool fill_grid,
                         const Mesh &src_mesh,
                         const GSpan src_value,
                         GMutableSpan dst_value)
{
  const CPPType &type = src_value.type();

  bke::attribute_math::convert_to_static_type(type, [&](auto dumpy) {
    using T = decltype(dumpy);
    const Span<T> src_typed_values = src_value.typed<T>();
    MutableSpan<T> dst_typed_values = dst_value.typed<T>();

    switch (domain) {
      case ATTR_DOMAIN_POINT: {
        if (fill_grid) {
          const IndexRange vert_mapping = builder.lookup_range("Vertices", "of Vertex");
          MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
          dst_vert_vertices.copy_from(src_typed_values);

          const OffsetIndices<int> edge_vertices_offsets = builder.lookup_offsets(
              "Vertices", "of Resampled Edge");
          MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
              builder.lookup_range("Vertices", "of Resampled Edge"));
          edges_have_created_new_verts<T>(
              edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);

          const IndexRange poly_vert_mapping = builder.lookup_range("Vertices",
                                                                    "of Grid Resampled Face");
          const OffsetIndices<int> face_vertices_offsets = builder.lookup_offsets(
              "Vertices", "of Grid Resampled Face");

          MutableSpan<T> poly_dst_vert_vertices = dst_typed_values.slice(poly_vert_mapping);
          polys_have_created_new_verts<T>(face_vertices_offsets,
                                          src_mesh.faces(),
                                          src_mesh.corner_verts(),
                                          src_mesh.corner_edges(),
                                          resample_edge_num,
                                          src_typed_values,
                                          poly_dst_vert_vertices);
        }
        else {
          const IndexRange vert_mapping = builder.lookup_range("Vertices", "of Vertex");
          MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
          dst_vert_vertices.copy_from(src_typed_values);

          const OffsetIndices<int> edge_vertices_offsets = builder.lookup_offsets(
              "Vertices", "of Resampled Edge");
          MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
              builder.lookup_range("Vertices", "of Resampled Edge"));
          edges_have_created_new_verts<T>(
              edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);
        }
        break;
      }
      case ATTR_DOMAIN_EDGE: {
        if (fill_grid) {
          const OffsetIndices<int> edge_edges_offsets = builder.lookup_offsets(
              "Edges", "of Resampled Edge");
          edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
          polys_have_created_new_edges<T>();
        }
        else {
          const OffsetIndices<int> edge_edges_offsets = builder.lookup_offsets(
              "Edges", "of Resampled Edge");
          edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
        }
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        if (fill_grid) {
          const OffsetIndices<int> loop_loops_offsets = builder.lookup_offsets(
              "Corners", "of Grid Resampled Face");
          loops_have_created_new_loops<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        }
        else {
          const OffsetIndices<int> loop_loops_offsets = builder.lookup_offsets(
              "Corners", "of Non-Grid Resampled Face");
          /* Loops and edges propagation is equal. */
          edges_have_created_new_edges<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        }
        break;
      }
      case ATTR_DOMAIN_FACE: {
        if (fill_grid) {
          polys_have_created_new_polys<T>(src_typed_values, dst_typed_values);
          polys_have_created_new_loops<T>();
        }
        else {
          dst_typed_values.copy_from(src_typed_values);
        }
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }
  });
}

}  // namespace propagation

namespace topology {

void build_face_loops(const bool left_right,
                      const IndexRange dst_edges_range,
                      const Span<int2> dst_edges_of_corner,
                      MutableSpan<int> r_dst_corner_verts,
                      MutableSpan<int> r_dst_corner_edges)
{
  if (left_right) {
    for (const int index : IndexRange(dst_edges_range.size())) {
      r_dst_corner_verts[index] = dst_edges_of_corner[index][0];
      r_dst_corner_edges[index] = dst_edges_range[index];
    }
  }
  else {
    for (const int i : IndexRange(dst_edges_range.size())) {
      const int index = dst_edges_range.size() - 1 - i;
      r_dst_corner_verts[index] = dst_edges_of_corner[i][1];
      r_dst_corner_edges[index] = dst_edges_range[i];
    }
  }
}

void build_faces_loops(const Span<int2> src_edges,
                       const Span<int> src_corner_verts,
                       const Span<int> src_corner_edges,
                       const OffsetIndices<int> dst_edge_offsets,
                       const OffsetIndices<int> dst_corner_offsets,
                       const Span<int2> dst_edges,
                       MutableSpan<int> r_corner_verts,
                       MutableSpan<int> r_corner_edges)
{
  for (const int corner_index : src_corner_verts.index_range()) {
    const int src_corner_vert = src_corner_verts[corner_index];
    const int src_corner_edge = src_corner_edges[corner_index];
    const int2 src_edge = src_edges[src_corner_edge];
    const IndexRange dst_edges_range = dst_edge_offsets[src_corner_edge];
    const bool left_right = src_edge[0] == src_corner_vert;

    const Span<int2> dst_edges_of_corner = dst_edges.slice(dst_edges_range);
    MutableSpan<int> dst_corner_verts = r_corner_verts.slice(dst_corner_offsets[corner_index]);
    MutableSpan<int> dst_corner_edges = r_corner_edges.slice(dst_corner_offsets[corner_index]);

    build_face_loops(
        left_right, dst_edges_range, dst_edges_of_corner, dst_corner_verts, dst_corner_edges);
  }
}

void build_faces(const OffsetIndices<int> src_polys,
                 const OffsetIndices<int> dst_corners,
                 MutableSpan<int> r_polys_offsets)
{
  for (const int index : src_polys.index_range()) {
    const IndexRange src_corners_range = src_polys[index];
    const int total_corners = dst_corners[src_corners_range].size();
    r_polys_offsets[index] = total_corners;
  }
  offset_indices::accumulate_counts_to_offsets(r_polys_offsets);
}

void build_edges(const OffsetIndices<int> dst_edges,
                 const OffsetIndices<int> dst_verts,
                 const IndexRange vert_mapping,
                 const Span<int2> src_edges,
                 MutableSpan<int2> r_edges)
{
  for (const int edge_index : src_edges.index_range()) {
    const int2 src_edge = src_edges[edge_index];
    MutableSpan<int2> edges = r_edges.slice(dst_edges[edge_index]);
    BLI_assert(!edges.is_empty());

    if (edges.size() == 1) {
      edges.first() = src_edge;
      continue;
    }

    const IndexRange verts_on_edge = dst_verts[edge_index];
    const IndexRange dst_left_verts = verts_on_edge.drop_back(1);
    const IndexRange dst_right_verts = verts_on_edge.drop_front(1);

    edges.first()[0] = src_edge[0];
    edges.first()[1] = vert_mapping[verts_on_edge.first()];

    const IndexRange edges_to_fill = edges.index_range().drop_front(1).drop_back(1);
    for (const int i : edges_to_fill.index_range()) {
      const int index = edges_to_fill[i];
      edges[index][0] = vert_mapping[dst_left_verts[i]];
      edges[index][1] = vert_mapping[dst_right_verts[i]];
    }

    edges.last()[0] = vert_mapping[verts_on_edge.last()];
    edges.last()[1] = src_edge[1];
  }
}

}  // namespace topology

void compute_mesh(const Mesh &mesh,
                  const Span<int> resample_edge_num,
                  const bool fill_grid,
                  builder::MeshBuilder &builder)
{
  /* New verices for original verts. */
  builder.push_element_by_size("Vertices", "of Vertex", mesh.totvert);

  /* New vertices and edges for original edges. */
  {
    VArray<int> new_vertices_for_original_edges = VArray<int>::ForSpan(resample_edge_num);
    VArray<int> new_edges_for_original_edges = VArray<int>::ForFunc(
        mesh.totedge,
        [resample_edge_num](const int64_t index) -> int { return resample_edge_num[index] + 1; });

    builder.push_virtual_element(
        "Vertices", "of Resampled Edge", std::move(new_vertices_for_original_edges));
    builder.push_virtual_element(
        "Edges", "of Resampled Edge", std::move(new_edges_for_original_edges));
  }

  if (fill_grid) {
    /* Face grid. */
    builder.push_element_by_size("Corners", "of Grid Resampled Face", 0);

    const OffsetIndices<int> src_polys = mesh.faces();
    const Span<int> src_loop_edges = mesh.corner_edges();

    VArray<int> new_verts_for_original_faces = VArray<int>::ForFunc(
        mesh.faces_num, [resample_edge_num, src_loop_edges, src_polys](const int poly_index) -> int {
          const Span<int> poly_edges = src_loop_edges.slice(src_polys[poly_index]);

          if (poly_edges.size() != 4) {
            return 0;
          }

          const int &points_num_edge_a = resample_edge_num[poly_edges[0]];
          const int &points_num_edge_b = resample_edge_num[poly_edges[1]];
          const int &points_num_edge_c = resample_edge_num[poly_edges[2]];
          const int &points_num_edge_d = resample_edge_num[poly_edges[3]];

          {
            const bool horizontal_equal = points_num_edge_a == points_num_edge_c;
            const bool vertical_equal = points_num_edge_b == points_num_edge_d;
            if (horizontal_equal && vertical_equal) {
              return points_num_edge_a * points_num_edge_b;
            }
          }

          const ResampleQuad resample(
              points_num_edge_a, points_num_edge_c, points_num_edge_b, points_num_edge_d);

          return resample.total_points();
        });

    builder.push_virtual_element(
        "Vertices", "of Grid Resampled Face", std::move(new_verts_for_original_faces));
  }
  else {
    /* New loops for original loops. */
    const Span<int> src_loop_edges = mesh.corner_edges();
    VArray<int> new_corners_for_original_faces = VArray<int>::ForFunc(
        src_loop_edges.size(), [resample_edge_num, src_loop_edges](const int corner_index) -> int {
          return resample_edge_num[src_loop_edges[corner_index]] + 1;
        });

    builder.push_virtual_element(
        "Corners", "of Non-Grid Resampled Face", std::move(new_corners_for_original_faces));

    /* New faces for original faces. */
    builder.push_element_by_size("Faces", "of Face N-gon", mesh.faces_num);
  }

  builder.finalize();
}

void build_topology(const Mesh &mesh,
                    const Span<int> /*resample_edge_num*/,
                    const bool fill_grid,
                    const builder::MeshBuilder &builder)
{
  using namespace topology;

  Mesh &result = builder.mesh();

  build_edges(builder.lookup_offsets("Edges", "of Resampled Edge"),
              builder.lookup_offsets("Vertices", "of Resampled Edge"),
              builder.lookup_range("Vertices", "of Resampled Edge"),
              mesh.edges(),
              result.edges_for_write().slice(builder.lookup_range("Edges", "of Resampled Edge")));

  if (fill_grid) {
    /* pass */
  }
  else {
    build_faces(mesh.faces(),
                builder.lookup_offsets("Corners", "of Non-Grid Resampled Face"),
                result.face_offsets_for_write());
    build_faces_loops(mesh.edges(),
                      mesh.corner_verts(),
                      mesh.corner_edges(),
                      builder.lookup_offsets("Edges", "of Resampled Edge"),
                      builder.lookup_offsets("Corners", "of Non-Grid Resampled Face"),
                      result.edges(),
                      result.corner_verts_for_write(),
                      result.corner_edges_for_write());
  }
}

static void propagate_attributes(const Mesh &src_mesh,
                                 const Span<int> resample_edge_num,
                                 const bool fill_grid,
                                 const builder::MeshBuilder &builder,
                                 const Map<bke::AttributeIDRef, bke::AttributeKind> &attributes)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = builder.mesh().attributes_for_write();

  for (Map<bke::AttributeIDRef, bke::AttributeKind>::Item entry : attributes.items()) {
    const bke::AttributeIDRef attribute_id = entry.key;
    bke::GAttributeReader attribute = src_attributes.lookup(attribute_id);

    if (!attribute) {
      continue;
    }

    if (!ELEM(attribute.domain,
              ATTR_DOMAIN_POINT,
              ATTR_DOMAIN_EDGE,
              ATTR_DOMAIN_CORNER,
              ATTR_DOMAIN_FACE))
    {
      continue;
    }

    const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(attribute.varray.type());
    bke::GSpanAttributeWriter result_attribute = dst_attributes.lookup_or_add_for_write_only_span(
        attribute_id, attribute.domain, data_type);

    if (!result_attribute) {
      continue;
    }

    GVArraySpan src_attribute_value(attribute.varray);
    GMutableSpan dst_attribute_value(result_attribute.span);

    using namespace propagation;
    attribute_on_domain(attribute.domain,
                        builder,
                        resample_edge_num,
                        fill_grid,
                        src_mesh,
                        src_attribute_value,
                        dst_attribute_value);

    result_attribute.finish();
  }
}

Mesh *resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const IndexMask &/*face_mask*/,
                        Map<bke::AttributeIDRef, bke::AttributeKind> attributes)
{
  builder::MeshBuilder builder;
  compute_mesh(mesh, resample_edge_num, false, builder);
  build_topology(mesh, resample_edge_num, false, builder);
  attributes.remove(".edge_verts");
  attributes.remove(".corner_edge");
  attributes.remove(".corner_vert");
  propagate_attributes(mesh, resample_edge_num, false, builder, attributes);

  return &builder.mesh();
}

}  // namespace blender::geometry
