/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_linear_allocator.hh"
#include "BLI_map.hh"
#include "BLI_offset_indices.hh"
#include "BLI_string_ref.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.h"

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

class MeshBuilder {
 private:
  LinearAllocator<> allocator_;

  struct OffsetRange {
    IndexRange range;
    std::optional<Span<int>> offsets;

    void status(std::stringstream &stream, const char *pref = "") const
    {
      stream << pref << "{Start: " << range.start();
      stream << " : Size: " << range.size();
      stream << ", End: " << range.one_after_last() << "}\n";
    }
  };

  struct Branch {
    Vector<OffsetRange> ordered_elements;
    Map<StringRef, OffsetRange *> named_elements;

    void status(std::stringstream &stream, const char *pref = "") const
    {
    }
  };

  Map<StringRef, Branch> mesh_primitives_;

  mutable Mesh *result = nullptr;

 public:
  MeshBuilder()
  {
    mesh_primitives_.reserve(4);
    mesh_primitives_.add_new("Vertices", {});
    mesh_primitives_.add_new("Edges", {});
    mesh_primitives_.add_new("Loops", {});
    mesh_primitives_.add_new("Faces", {});
  }

  void push_element(const StringRef &primitive_type,
                    const StringRef &identifier,
                    const OffsetRange &element)
  {
    Branch &branch = mesh_primitives_.lookup(primitive_type);
    if (branch.ordered_elements.is_empty()) {
      BLI_assert(branch.named_elements.is_empty());
      branch.ordered_elements.append(element);
      branch.named_elements.add_new(identifier, &branch.ordered_elements.first());
      return;
    }

    const OffsetRange &previous_element = branch.ordered_elements.last();
    const int64_t n_shift = previous_element.range.one_after_last();

    OffsetRange shifted_element = element;
    shifted_element.range = shifted_element.range.shift(n_shift);

    const int index = branch.ordered_elements.append_and_get_index(shifted_element);
    branch.named_elements.add_new(identifier, &branch.ordered_elements[index]);
  }

  void push_element_by_size(const StringRef &primitive_type,
                            const StringRef identifier,
                            const int total)
  {
    this->push_element(primitive_type, identifier, OffsetRange{IndexRange(total), std::nullopt});
  }

  void push_virtual_element(const StringRef primitive_type,
                            const StringRef identifier,
                            const VArray<int> counts)
  {
    if (counts.is_single()) {
      const int total_size = counts.get_internal_single() * counts.size();
      this->push_element_by_size(primitive_type, identifier, total_size);
      return;
    }

    MutableSpan<int> accumulations = allocator_.allocate_array<int>(counts.size() + 1);
    accumulations.last() = 0;
    counts.materialize(accumulations.drop_back(1));
    offset_indices::accumulate_counts_to_offsets(accumulations);
    const int total_size = accumulations.last();

    OffsetRange new_element{IndexRange(total_size), accumulations.as_span()};
    this->push_element(primitive_type, identifier, new_element);
  }

  IndexRange lookup_range(const StringRef primitive_type, const StringRef identifier) const
  {
    const Branch &branch = mesh_primitives_.lookup(primitive_type);
    const OffsetRange &offset_range = *branch.named_elements.lookup(identifier);
    return offset_range.range;
  }

  OffsetIndices<int> lookup_offsets(const StringRef primitive_type,
                                    const StringRef identifier) const
  {
    const Branch &branch = mesh_primitives_.lookup(primitive_type);
    const OffsetRange &offset_range = *branch.named_elements.lookup(identifier);
    BLI_assert(offset_range.offsets.has_value());
    return OffsetIndices<int>(*offset_range.offsets);
  }

  void finalize()
  {
    BLI_assert(result == nullptr);

    const auto branch_total = [this](const StringRef primitive_type) {
      const Branch &branch = mesh_primitives_.lookup(primitive_type);
      if (branch.ordered_elements.is_empty()) {
        return 0;
      }
      const int size = branch.ordered_elements.last().range.one_after_last();
      return size;
    };

    const int tot_vert = branch_total("Vertices");
    const int tot_edge = branch_total("Edges");
    const int tot_loop = branch_total("Loops");
    const int tot_face = branch_total("Faces");

    result = BKE_mesh_new_nomain(tot_vert, tot_edge, tot_loop, tot_face);
  }

  Mesh &mesh() const
  {
    BLI_assert(result != nullptr);
    return *result;
  }

  void status(std::stringstream &stream) const
  {
    stream << "Status:\n";
    stream << "  Has mesh: " << ((result != nullptr) ? "True" : "False") << ";\n";
    stream << "  Customs{\n";
    mesh_primitives_.lookup("Vertices").status(stream, "Vertices");
    mesh_primitives_.lookup("Edges").status(stream, "Edges");
    mesh_primitives_.lookup("Loops").status(stream, "Loops");
    mesh_primitives_.lookup("Faces").status(stream, "Faces");
    stream << "};\n";
  }
};

namespace propagation {

template<typename T>
void edges_have_created_new_verts(const OffsetIndices<int> offsets,
                                  const Span<MEdge> edges,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{
  for (const int index : offsets.index_range()) {
    const MEdge edge = edges[index];

    const T value_on_a_edge_vertex = src[edge.v1];
    const T value_on_b_edge_vertex = src[edge.v2];

    const IndexRange range = offsets[index];
    const int size = range.size();

    for (const int index_in_edge : IndexRange(size)) {
      const float factor = float(index_in_edge + 1) / float(size + 1);
      dst[range.start() + index_in_edge] = attribute_math::mix2(
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
void polys_have_created_new_verts(const OffsetIndices<int> offsets,
                                  const Span<MPoly> polys,
                                  const Span<MLoop> loops,
                                  const Span<int> resample_edge_num,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{

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
      */
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

        dst[point_index] = attribute_math::mix2(
            uv_point.x,
            attribute_math::mix2(uv_point.y, src[points_a], src[points_b]),
            attribute_math::mix2(uv_point.y, src[points_d], src[points_c]));

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

        dst[point_index] = attribute_math::mix2(
            uv_point.x,
            attribute_math::mix2(uv_point.y, src[points_a], src[points_b]),
            attribute_math::mix2(uv_point.y, src[points_d], src[points_c]));

        point_index++;
      }
    }
  }
}

template<typename T> void polys_have_created_new_edges()
{
}

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

template<typename T> void polys_have_created_new_loops()
{
}

void attribute_on_domain(const eAttrDomain domain,
                         const MeshBuilder &builder,
                         const Span<int> resample_edge_num,
                         const bool try_to_fill_by_grid,
                         const Mesh &src_mesh,
                         const GSpan src_value,
                         GMutableSpan dst_value)
{
  const CPPType &type = src_value.type();

  attribute_math::convert_to_static_type(type, [&](auto dumpy) {
    using T = decltype(dumpy);
    const Span<T> src_typed_values = src_value.typed<T>();
    MutableSpan<T> dst_typed_values = dst_value.typed<T>();

    switch (domain) {
      case ATTR_DOMAIN_POINT: {
        if (try_to_fill_by_grid) {
          const IndexRange vert_mapping = builder.lookup_range("Vertices", "of Vertex");
          MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
          dst_vert_vertices.copy_from(src_typed_values);

          const OffsetIndices<int> edge_vertices_offsets = builder.lookup_offsets("Vertices",
                                                                                  "of Edge");
          MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
              builder.lookup_range("Vertices", "of Edge"));
          edges_have_created_new_verts<T>(
              edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);

          const IndexRange poly_vert_mapping = builder.lookup_range("Vertices", "of Face Grid");
          const OffsetIndices<int> face_vertices_offsets = builder.lookup_offsets("Vertices",
                                                                                  "of Face Grid");

          MutableSpan<T> poly_dst_vert_vertices = dst_typed_values.slice(poly_vert_mapping);
          polys_have_created_new_verts<T>(face_vertices_offsets,
                                          src_mesh.polys(),
                                          src_mesh.loops(),
                                          resample_edge_num,
                                          src_typed_values,
                                          poly_dst_vert_vertices);
        }
        else {
          const IndexRange vert_mapping = builder.lookup_range("Vertices", "of Vertex");
          MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
          dst_vert_vertices.copy_from(src_typed_values);

          const OffsetIndices<int> edge_vertices_offsets = builder.lookup_offsets("Vertices",
                                                                                  "of Edge");
          MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
              builder.lookup_range("Vertices", "of Edge"));
          edges_have_created_new_verts<T>(
              edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);
        }
        break;
      }
      case ATTR_DOMAIN_EDGE: {
        if (try_to_fill_by_grid) {
          const OffsetIndices<int> edge_edges_offsets = builder.lookup_offsets("Edges", "of Edge");
          edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
          polys_have_created_new_edges<T>();
        }
        else {
          const OffsetIndices<int> edge_edges_offsets = builder.lookup_offsets("Edges", "of Edge");
          edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
        }
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        if (try_to_fill_by_grid) {
          const OffsetIndices<int> loop_loops_offsets = builder.lookup_offsets("Loops",
                                                                               "of Face Grid");
          loops_have_created_new_loops<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        }
        else {
          const OffsetIndices<int> loop_loops_offsets = builder.lookup_offsets(
              "Loops", "of Face N-gon Loop");
          /* Loops and edges propagation is equal. */
          edges_have_created_new_edges<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        }
        break;
      }
      case ATTR_DOMAIN_FACE: {
        if (try_to_fill_by_grid) {
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

void build_face_loops(const bool flip_order,
                      const IndexRange edge_mappng,
                      const Span<MEdge> dst_edges,
                      MutableSpan<MLoop> dst_loops_r)
{
  if (flip_order) {
    for (const int index : IndexRange(edge_mappng.size())) {
      dst_loops_r[index].e = edge_mappng[index];
      dst_loops_r[index].v = dst_edges[index].v1;
    }
  }
  else {
    for (const int i : IndexRange(edge_mappng.size())) {
      const int index = edge_mappng.size() - 1 - i;
      dst_loops_r[index].e = edge_mappng[i];
      dst_loops_r[index].v = dst_edges[i].v2;
    }
  }
}

void build_faces_loops(const Span<MEdge> src_edges,
                       const Span<MLoop> src_loops,
                       const OffsetIndices<int> dst_edges,
                       const OffsetIndices<int> dst_loops,
                       const Span<MEdge> dst_edge_value,
                       MutableSpan<MLoop> r_dst_loops)
{
  for (const int loop_index : src_loops.index_range()) {
    const MLoop src_loop = src_loops[loop_index];
    const MEdge src_edge = src_edges[src_loop.e];
    const IndexRange dst_edge = dst_edges[src_loop.e];
    const bool flip_order = src_edge.v1 == src_loop.v;

    const Span<MEdge> dst_loop_edges = dst_edge_value.slice(dst_edge);
    MutableSpan<MLoop> dst_loops_r = r_dst_loops.slice(dst_loops[loop_index]);

    build_face_loops(flip_order, dst_edge, dst_loop_edges, dst_loops_r);
  }
}

void build_faces(const Span<MPoly> src_polys,
                 const OffsetIndices<int> dst_loop,
                 MutableSpan<MPoly> dst_polys)
{
  for (const int index : src_polys.index_range()) {
    const MPoly src_poly = src_polys[index];

    const IndexRange src_loops(src_poly.loopstart, src_poly.totloop);

    const int loopstart = dst_loop[src_loops.first()].start();
    const int totloop = dst_loop[src_loops.last()].one_after_last() - loopstart;

    MPoly &dst_poly = dst_polys[index];

    dst_poly.loopstart = loopstart;
    dst_poly.totloop = totloop;
  }
}

void build_edge_vert_indices(const OffsetIndices<int> new_edges,
                             const OffsetIndices<int> new_verts_for_edges,
                             const Span<MEdge> edges,
                             const IndexRange vert_mapping,
                             MutableSpan<MEdge> r_edges)
{
  for (const int index : new_edges.index_range()) {
    const MEdge src_edge = edges[index];
    MutableSpan<MEdge> dst_edges = r_edges.slice(new_edges[index]);

    if (dst_edges.size() == 1) {
      dst_edges.first().v1 = src_edge.v1;
      dst_edges.first().v2 = src_edge.v2;
      continue;
    }

    const IndexRange dst_vertices = new_verts_for_edges[index];

    int v1 = -1;
    int v2 = src_edge.v1;
    for (const int i : dst_edges.drop_back(1).index_range()) {
      v1 = v2;
      v2 = vert_mapping[dst_vertices[i]];
      dst_edges[i].v1 = v1;
      dst_edges[i].v2 = v2;
    }

    dst_edges.last().v1 = vert_mapping[dst_vertices.last()];
    dst_edges.last().v2 = src_edge.v2;
  }
}

}  // namespace topology

void compute_mesh(const Mesh &mesh,
                  const Span<int> resample_edge_num,
                  const bool try_to_fill_by_grid,
                  MeshBuilder &builder)
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
        "Vertices", "of Edge", std::move(new_vertices_for_original_edges));
    builder.push_virtual_element("Edges", "of Edge", std::move(new_edges_for_original_edges));
  }

  if (try_to_fill_by_grid) {
    /* Face grid. */
    builder.push_element_by_size("Loops", "of Face Grid", 0);

    const Span<MLoop> src_loops = mesh.loops();
    const Span<MPoly> src_polys = mesh.polys();

    VArray<int> new_verts_for_original_faces = VArray<int>::ForFunc(
        mesh.totpoly, [resample_edge_num, src_loops, src_polys](const int64_t index) -> int {
          const MPoly face = src_polys[index];
          const IndexRange loop_range(face.loopstart, face.totloop);
          const Span<MLoop> loops = src_loops.slice(loop_range);

          if (loops.size() != 4) {
            return 0;
          }

          const int &points_num_edge_a = resample_edge_num[loops[0].e];
          const int &points_num_edge_b = resample_edge_num[loops[1].e];
          const int &points_num_edge_c = resample_edge_num[loops[2].e];
          const int &points_num_edge_d = resample_edge_num[loops[3].e];

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
        "Vertices", "of Face Grid", std::move(new_verts_for_original_faces));
  }
  else {
    /* New loops for original loops. */
    const Span<MLoop> src_loops = mesh.loops();
    VArray<int> new_loops_for_original_faces = VArray<int>::ForFunc(
        mesh.totloop, [resample_edge_num, src_loops](const int64_t index) -> int {
          const MLoop src_loop = src_loops[index];
          return resample_edge_num[src_loop.e] + 1;
        });

    builder.push_virtual_element(
        "Loops", "of Face N-gon Loop", std::move(new_loops_for_original_faces));

    /* New faces for original faces. */
    builder.push_element_by_size("Faces", "of Face N-gon", mesh.totpoly);
  }

  builder.finalize();
}

void build_topology(const Mesh &mesh,
                    const Span<int> /*resample_edge_num*/,
                    const bool try_to_fill_by_grid,
                    const MeshBuilder &builder)
{
  using namespace topology;

  Mesh &result = builder.mesh();

  build_edge_vert_indices(
      builder.lookup_offsets("Edges", "of Edge"),
      builder.lookup_offsets("Vertices", "of Edge"),
      mesh.edges(),
      builder.lookup_range("Vertices", "of Edge"),
      result.edges_for_write().slice(builder.lookup_range("Edges", "of Edge")));

  if (try_to_fill_by_grid) {
    /* pass */
  }
  else {
    build_faces_loops(mesh.edges(),
                      mesh.loops(),
                      builder.lookup_offsets("Edges", "of Edge"),
                      builder.lookup_offsets("Loops", "of Face N-gon Loop"),
                      result.edges(),
                      result.loops_for_write());
    build_faces(mesh.polys(),
                builder.lookup_offsets("Loops", "of Face N-gon Loop"),
                result.polys_for_write());
  }
}

static void propagate_attributes(const Mesh &src_mesh,
                                 const Span<int> resample_edge_num,
                                 const bool try_to_fill_by_grid,
                                 const MeshBuilder &builder,
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
              ATTR_DOMAIN_FACE)) {
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
                        try_to_fill_by_grid,
                        src_mesh,
                        src_attribute_value,
                        dst_attribute_value);

    result_attribute.finish();
  }
}

Mesh &resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const bool try_to_fill_by_grid,
                        const Map<bke::AttributeIDRef, bke::AttributeKind> attributes)
{
  MeshBuilder builder;
  compute_mesh(mesh, resample_edge_num, try_to_fill_by_grid, builder);
  build_topology(mesh, resample_edge_num, try_to_fill_by_grid, builder);
  propagate_attributes(mesh, resample_edge_num, try_to_fill_by_grid, builder, attributes);

  return builder.mesh();
}

}  // namespace blender::geometry
