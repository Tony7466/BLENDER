/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_linear_allocator.hh"
#include "BLI_offset_indices.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.h"

#include "GEO_mesh_resample_topology.hh"

class ResampleQuad {
 private:
  /*
  struct EdgePair {
    const int splittings;
    const int edges_a;
    const int edges_b;
    const int max_edges;
    const int min_edges;
    const int corner;
    const int body;
  };
  */
  const int hor_conections_;
  const int ver_conections_;

  const int left_edges_;
  const int right_edges_;
  const int top_edges_;
  const int bottom_edges_;

  const int hor_max_;
  const int ver_max_;

  const int hor_min_;
  const int ver_min_;

  const int hor_corner_total_;
  const int ver_corner_total_;

  const int hor_body_total_;
  const int ver_body_total_;

 public:
  ResampleQuad(const int left_added_points,
               const int right_added_points,
               const int top_added_points,
               const int bottom_added_points)
      : hor_conections_(split_lines_between(top_added_points, bottom_added_points)),
        ver_conections_(split_lines_between(left_added_points, right_added_points)),
        left_edges_(left_added_points + 1),
        right_edges_(right_added_points + 1),
        top_edges_(top_added_points + 1),
        bottom_edges_(bottom_added_points + 1),
        hor_max_(std::max(top_edges_, bottom_edges_)),
        ver_max_(std::max(left_edges_, right_edges_)),
        hor_min_(std::min(top_edges_, bottom_edges_)),
        ver_min_(std::min(left_edges_, right_edges_)),
        hor_corner_total_(corner_for_size(hor_max_, hor_min_)),
        ver_corner_total_(corner_for_size(ver_max_, ver_min_)),
        hor_body_total_(body_of_side_without_corners(hor_conections_, hor_corner_total_)),
        ver_body_total_(body_of_side_without_corners(ver_conections_, ver_corner_total_))
  {
  }

  int total_points() const
  {
    const int body_body_points = hor_body_total_ * ver_body_total_;

    const int body_corner_points_hor = hor_body_total_ * ver_corner_total_;
    const int body_corner_points_ver = ver_body_total_ * hor_corner_total_;

    const int side_corner_points_hor = hor_conections_ * ver_corner_total_;
    const int side_corner_points_ver = ver_conections_ * hor_corner_total_;

    const int corner_corner_duply_sub = hor_corner_total_ * ver_corner_total_;

    const int total_points = body_body_points + body_corner_points_hor + body_corner_points_ver +
                             side_corner_points_hor + side_corner_points_ver -
                             corner_corner_duply_sub;

    BLI_assert(total_points >= 0);
    return total_points;
  }

 protected:
  /*
   * A     -     -     B
   * |\  / | \ / | \ / | <-|
   * C - - - - - - - - D
   * Is number of edge-lines for side pair without bounding pair.
   */
  int split_lines_between(const int &tot_points_a, const int &tot_points_b) const
  {
    return std::max(tot_points_a, tot_points_b);
  }

  /*
   * A     -     -     B
   * |\  / | \ / | \ / |
   * C - - - - - - - - D
   *   ^             ^
   * Is points of side corner.
   */
  int corner_for_size(const int &edge_max, const int &edge_min) const
  {
    return edge_max / (edge_min * 2);
  }

  /*
   * A     -     -     B
   * |\  / | \ / | \ / |
   * C - - - - - - - - D
   *     ^ ^ ^ ^ ^ ^
   * Is points of side body.
   */
  int body_of_side_without_corners(const int &side, const int &corner) const
  {
    return side - corner * 2;
  }
};

namespace blender::detail {

class MeshMarket {
 private:
  LinearAllocator<> allocator_;

  struct OffestRange {
    IndexRange range;
    std::optional<Span<int>> offsets;

    OffestRange() = default;

    OffestRange(const IndexRange in_range, const Span<int> in_offsets)
        : range(in_range), offsets(in_offsets)
    {
    }

    OffestRange(const IndexRange in_range) : range(in_range)
    {
    }

    OffestRange shift(const int64_t n) const
    {
      if (offsets.has_value()) {
        return OffestRange(range.shift(n), *offsets);
      }
      return OffestRange(range.shift(n));
    }

    void status(std::stringstream &stream, const char *pref = "") const
    {
      stream << pref << "{Start: " << range.start();
      stream << " : Size: " << range.size();
      stream << ", End: " << range.one_after_last() << "}\n";
    }
  };

  struct CustomData {
    OffestRange vert;
    OffestRange edge;
    OffestRange loop;
    OffestRange face;

    void status(std::stringstream &stream, const char *pref = "") const
    {
      stream << pref << "CustomData:\n";
      {
        std::string new_pref = std::string(pref) + std::string("  Verts");
        vert.status(stream, new_pref.c_str());
      }
      {
        std::string new_pref = std::string(pref) + std::string("  Edges");
        edge.status(stream, new_pref.c_str());
      }
      {
        std::string new_pref = std::string(pref) + std::string("  Loops");
        loop.status(stream, new_pref.c_str());
      }
      {
        std::string new_pref = std::string(pref) + std::string("  Faces");
        face.status(stream, new_pref.c_str());
      }
    };
  };

  Vector<CustomData> customs_;

  mutable Mesh *result = nullptr;

 public:
  MeshMarket() = default;

  void status(std::stringstream &stream) const
  {
    stream << "Status:\n";
    stream << "  Has mesh: " << ((result != nullptr) ? "True" : "False") << ";\n";
    stream << "  Customs{\n";
    for (const CustomData &cdata : customs_) {
      cdata.status(stream, "    ");
    }
    stream << "};\n";
  }

  int add_custom(const int tot_vert, const int tot_edge, const int tot_loop, const int tot_face)
  {
    const int index = this->add_custom(OffestRange(IndexRange(tot_vert)),
                                       OffestRange(IndexRange(tot_edge)),
                                       OffestRange(IndexRange(tot_loop)),
                                       OffestRange(IndexRange(tot_face)));
    return index;
  }

  int add_custom(const OffestRange vert,
                 const OffestRange edge,
                 const OffestRange loop,
                 const OffestRange face)
  {
    const CustomData new_range{vert, edge, loop, face};

    if (customs_.is_empty()) {
      customs_.append(new_range);
      return 0;
    }

    const CustomData previous_one = customs_.last();
    const CustomData another_one{new_range.vert.shift(previous_one.vert.range.one_after_last()),
                                 new_range.edge.shift(previous_one.edge.range.one_after_last()),
                                 new_range.loop.shift(previous_one.loop.range.one_after_last()),
                                 new_range.face.shift(previous_one.face.range.one_after_last())};

    return customs_.append_and_get_index(another_one);
  }

  int add_custom(const CustomData custom)
  {
    const int index = this->add_custom(custom.vert, custom.edge, custom.loop, custom.face);
    return index;
  }

  int add_custom(const VArray<int> verts,
                 const VArray<int> edges,
                 const VArray<int> loops,
                 const VArray<int> faces)
  {

    auto total_size = [&allocator_ = this->allocator_](const VArray<int> &counts,
                                                       OffestRange &r_offset_range) {
      if (counts.is_single()) {
        const int total_size = counts.get_internal_single() * counts.size();
        r_offset_range.range = IndexRange(total_size);
        return;
      }
      else {
        MutableSpan<int> accumulations = allocator_.allocate_array<int>(counts.size() + 1);
        accumulations.last() = 0;
        counts.materialize(accumulations.drop_back(1));
        offset_indices::accumulate_counts_to_offsets(accumulations);
        const int total_size = accumulations.last();

        r_offset_range.range = IndexRange(total_size);
        r_offset_range.offsets.emplace(accumulations.as_span());
        return;
      }
    };

    CustomData custom;

    total_size(std::move(verts), custom.vert);
    total_size(std::move(edges), custom.edge);
    total_size(std::move(loops), custom.loop);
    total_size(std::move(faces), custom.face);

    return this->add_custom(custom);
  }

  void finalize()
  {
    BLI_assert(result == nullptr);
    const CustomData &last_custom = customs_.last();
    result = BKE_mesh_new_nomain(last_custom.vert.range.one_after_last(),
                                 last_custom.edge.range.one_after_last(),
                                 0,
                                 last_custom.loop.range.one_after_last(),
                                 last_custom.face.range.one_after_last());
  }

  Mesh &mesh() const
  {
    BLI_assert(result != nullptr);
    return *result;
  }

  IndexRange get_vert_range_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    return custom.vert.range;
  }

  IndexRange get_edge_range_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    return custom.edge.range;
  }

  IndexRange get_loop_range_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    return custom.loop.range;
  }

  IndexRange get_face_range_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    return custom.face.range;
  }

  OffsetIndices<int> get_vert_offsets_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    BLI_assert(custom.vert.offsets.has_value());
    return *custom.vert.offsets;
  }

  OffsetIndices<int> get_edge_offsets_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    BLI_assert(custom.edge.offsets.has_value());
    return *custom.edge.offsets;
  }

  OffsetIndices<int> get_loop_offsets_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    BLI_assert(custom.loop.offsets.has_value());
    return *custom.loop.offsets;
  }

  OffsetIndices<int> get_face_offsets_in(const int index) const
  {
    const CustomData &custom = customs_[index];
    BLI_assert(custom.face.offsets.has_value());
    return *custom.face.offsets;
  }
};

}  // namespace blender::detail

namespace blender::geometry {

namespace propagation_for_grid {

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
void polys_have_created_new_verts(const OffsetIndices<int> offsets,
                                  const Span<MPoly> polys,
                                  const Span<MLoop> loops,
                                  const Span<int> resample_edge_num,
                                  const Span<T> src,
                                  MutableSpan<T> dst)
{
  for (const int poly_i : polys.index_range()) {
    const MPoly src_poly = polys[poly_i];
    const Span<MLoop> src_loops = loops.slice(src_poly.loopstart, src_poly.totloop);
    const IndexRange poly_vertices = offsets[poly_i];

    if (src_loops.size() == 3) {
      return;
    }
    else if (src_loops.size() == 4) {
      const int &points_a = src_loops[0].v;
      const int &points_b = src_loops[1].v;
      const int &points_c = src_loops[2].v;
      const int &points_d = src_loops[3].v;

      const int &points_num_edge_a = resample_edge_num[src_loops[0].e];
      const int &points_num_edge_b = resample_edge_num[src_loops[1].e];
      const int &points_num_edge_c = resample_edge_num[src_loops[2].e];
      const int &points_num_edge_d = resample_edge_num[src_loops[3].e];

      const int horizontal_connections = math::max(points_num_edge_a, points_num_edge_c);
      const int horizontal_other_side = math::min(points_num_edge_a, points_num_edge_c);

      const int vertical_connections = math::max(points_num_edge_b, points_num_edge_d);
      const int vertical_other_side = math::min(points_num_edge_b, points_num_edge_d);

      const auto find_nearest_one = [](const int max, const int min, const int i) -> int {
        return i * min / max;
      };

      const auto intersection =
          [](const float2 p1, const float2 p2, const float2 p3, const float2 p4) -> float2 {
        printf(">\n");
        printf(" - x1: %f, y1: %f;\n", p1.x, p1.y);
        printf(" - x2: %f, y2: %f;\n", p2.x, p2.y);
        printf(" - x3: %f, y3: %f;\n", p3.x, p3.y);
        printf(" - x4: %f, y4: %f;\n", p4.x, p4.y);
        return {};
      };

      for (const int x : IndexRange(horizontal_connections)) {
        const int x_other = find_nearest_one(horizontal_connections, horizontal_other_side, x);

        const float x_p_left = float(x * horizontal_other_side);
        const float x_p_right = float(x_other * horizontal_connections);

        for (const int y : IndexRange(vertical_connections)) {
          const int y_other = find_nearest_one(vertical_connections, vertical_other_side, y);

          const float y_p_top = float(y * vertical_other_side);
          const float y_p_bottom = float(y_other * vertical_connections);
          return;
          const float2 uv = intersection(float2{0.0f, x_p_left},
                                         float2{1.0f, x_p_right},
                                         float2{y_p_top, 0.0f},
                                         float2{y_p_bottom, 1.0f});

          attribute_math::mix2(uv.x,
                               attribute_math::mix2(uv.y, src[points_a], src[points_b]),
                               attribute_math::mix2(uv.y, src[points_c], src[points_d]));
        }
      }
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
                         const detail::MeshMarket &mesh_builder,
                         const Span<int> resample_edge_num,
                         const Mesh &src_mesh,
                         const GSpan src_value,
                         Mesh & /*dst_mesh*/,
                         GMutableSpan dst_value)
{
  const CPPType &type = src_value.type();

  attribute_math::convert_to_static_type(type, [&](auto dumpy) {
    using T = decltype(dumpy);
    const Span<T> src_typed_values = src_value.typed<T>();
    MutableSpan<T> dst_typed_values = dst_value.typed<T>();

    switch (domain) {
      case ATTR_DOMAIN_POINT: {
        const IndexRange vert_mapping = mesh_builder.get_vert_range_in(0);
        MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
        dst_vert_vertices.copy_from(src_typed_values);

        const OffsetIndices<int> edge_vertices_offsets = mesh_builder.get_vert_offsets_in(1);
        MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
            mesh_builder.get_vert_range_in(1));
        edges_have_created_new_verts<T>(
            edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);

        const IndexRange poly_vert_mapping = mesh_builder.get_vert_range_in(2);
        const OffsetIndices<int> face_vertices_offsets = mesh_builder.get_vert_offsets_in(2);

        MutableSpan<T> poly_dst_vert_vertices = dst_typed_values.slice(poly_vert_mapping);
        polys_have_created_new_verts<T>(face_vertices_offsets,
                                        src_mesh.polys(),
                                        src_mesh.loops(),
                                        resample_edge_num,
                                        src_typed_values,
                                        poly_dst_vert_vertices);
        break;
      }
      case ATTR_DOMAIN_EDGE: {
        const OffsetIndices<int> edge_edges_offsets = mesh_builder.get_edge_offsets_in(1);
        edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
        polys_have_created_new_edges<T>();
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        const OffsetIndices<int> loop_loops_offsets = mesh_builder.get_loop_offsets_in(2);
        loops_have_created_new_loops<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        break;
      }
      case ATTR_DOMAIN_FACE: {
        polys_have_created_new_polys<T>(src_typed_values, dst_typed_values);
        polys_have_created_new_loops<T>();
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }
  });
}

}  // namespace propagation_for_grid

namespace propagation_for_ngone {

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

void attribute_on_domain(const eAttrDomain domain,
                         const detail::MeshMarket &mesh_builder,
                         const Mesh &src_mesh,
                         const GSpan src_value,
                         Mesh & /*dst_mesh*/,
                         GMutableSpan dst_value)
{
  const CPPType &type = src_value.type();

  attribute_math::convert_to_static_type(type, [&](auto dumpy) {
    using T = decltype(dumpy);
    const Span<T> src_typed_values = src_value.typed<T>();
    MutableSpan<T> dst_typed_values = dst_value.typed<T>();

    switch (domain) {
      case ATTR_DOMAIN_POINT: {
        const IndexRange vert_mapping = mesh_builder.get_vert_range_in(0);
        MutableSpan<T> dst_vert_vertices = dst_typed_values.slice(vert_mapping);
        dst_vert_vertices.copy_from(src_typed_values);

        const OffsetIndices<int> edge_vertices_offsets = mesh_builder.get_vert_offsets_in(1);
        MutableSpan<T> dst_edge_vertices = dst_typed_values.slice(
            mesh_builder.get_vert_range_in(1));
        edges_have_created_new_verts<T>(
            edge_vertices_offsets, src_mesh.edges(), src_typed_values, dst_edge_vertices);
        break;
      }
      case ATTR_DOMAIN_EDGE: {
        const OffsetIndices<int> edge_edges_offsets = mesh_builder.get_edge_offsets_in(1);
        edges_have_created_new_edges<T>(edge_edges_offsets, src_typed_values, dst_typed_values);
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        const OffsetIndices<int> loop_loops_offsets = mesh_builder.get_loop_offsets_in(2);
        /* Loops and edges propagation is equal. */
        edges_have_created_new_edges<T>(loop_loops_offsets, src_typed_values, dst_typed_values);
        break;
      }
      case ATTR_DOMAIN_FACE: {
        dst_typed_values.copy_from(src_typed_values);
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }
  });
}

}  // namespace propagation_for_ngone

namespace ngone_fill {

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

}  // namespace ngone_fill

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

static void propagate_attributes_on_new_mesh(
    const detail::MeshMarket &mesh_builder,
    const Span<int> resample_edge_num,
    const bool try_to_fill_by_grid,
    const Map<bke::AttributeIDRef, bke::AttributeKind> &attributes,
    const Mesh &src_mesh)
{
  Mesh &result = mesh_builder.mesh();

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = result.attributes_for_write();

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

    if (try_to_fill_by_grid) {
      using namespace propagation_for_grid;
      attribute_on_domain(attribute.domain,
                          mesh_builder,
                          resample_edge_num,
                          src_mesh,
                          src_attribute_value,
                          result,
                          dst_attribute_value);
    }
    else {
      using namespace propagation_for_ngone;
      attribute_on_domain(attribute.domain,
                          mesh_builder,
                          src_mesh,
                          src_attribute_value,
                          result,
                          dst_attribute_value);
    }

    result_attribute.finish();
  }
}

void build_mesh(const Mesh &mesh,
                const Span<int> resample_edge_num,
                const bool try_to_fill_by_grid,
                detail::MeshMarket &r_mesh_builder)
{
  /* New verices for original verts. */
  r_mesh_builder.add_custom(mesh.totvert, 0, 0, 0);

  /* New vertices and edges for original edges. */
  {
    VArray<int> new_vertices_for_original_edges = VArray<int>::ForSpan(resample_edge_num);
    VArray<int> new_edges_for_original_edges = VArray<int>::ForFunc(
        mesh.totedge,
        [resample_edge_num](const int64_t index) -> int { return resample_edge_num[index] + 1; });

    r_mesh_builder.add_custom(std::move(new_vertices_for_original_edges),
                              std::move(new_edges_for_original_edges),
                              VArray<int>::ForSingle(0, 0),
                              VArray<int>::ForSingle(0, 0));
  }

  if (try_to_fill_by_grid) {
    /* Face grid. */
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

          ResampleQuad resample(
              points_num_edge_a, points_num_edge_c, points_num_edge_b, points_num_edge_d);

          return resample.total_points();
        });

    r_mesh_builder.add_custom(std::move(new_verts_for_original_faces),
                              VArray<int>::ForSingle(0, 0),
                              VArray<int>::ForSingle(0, 0),
                              VArray<int>::ForSingle(0, 0));
  }
  else {
    /* New loops for original loops. */
    {
      const Span<MLoop> src_loops = mesh.loops();
      VArray<int> new_loops_for_original_faces = VArray<int>::ForFunc(
          mesh.totloop, [resample_edge_num, src_loops](const int64_t index) -> int {
            const MLoop src_loop = src_loops[index];
            return resample_edge_num[src_loop.e] + 1;
          });

      r_mesh_builder.add_custom(VArray<int>::ForSingle(0, 0),
                                VArray<int>::ForSingle(0, 0),
                                std::move(new_loops_for_original_faces),
                                VArray<int>::ForSingle(0, 0));
    }
    /* New faces for original faces. */
    r_mesh_builder.add_custom(0, 0, 0, mesh.totpoly);
  }

  r_mesh_builder.finalize();
}

void build_new_mesh_topology(const Mesh &mesh,
                             const Span<int> /*resample_edge_num*/,
                             const bool try_to_fill_by_grid,
                             const detail::MeshMarket &mesh_builder)
{
  Mesh &result = mesh_builder.mesh();
  build_edge_vert_indices(mesh_builder.get_edge_offsets_in(1),
                          mesh_builder.get_vert_offsets_in(1),
                          mesh.edges(),
                          mesh_builder.get_vert_range_in(1),
                          result.edges_for_write().slice(mesh_builder.get_edge_range_in(1)));

  if (try_to_fill_by_grid) {
    return;
  }
  else {
    using namespace ngone_fill;
    build_faces_loops(mesh.edges(),
                      mesh.loops(),
                      mesh_builder.get_edge_offsets_in(1),
                      mesh_builder.get_loop_offsets_in(2),
                      result.edges(),
                      result.loops_for_write());
    build_faces(mesh.polys(), mesh_builder.get_loop_offsets_in(2), result.polys_for_write());
  }
}

Mesh &resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const bool try_to_fill_by_grid,
                        const Map<bke::AttributeIDRef, bke::AttributeKind> attributes)
{
  detail::MeshMarket mesh_builder;
  build_mesh(mesh, resample_edge_num, try_to_fill_by_grid, mesh_builder);

  build_new_mesh_topology(mesh, resample_edge_num, try_to_fill_by_grid, mesh_builder);

  propagate_attributes_on_new_mesh(
      mesh_builder, resample_edge_num, try_to_fill_by_grid, attributes, mesh);

  return mesh_builder.mesh();
}

}  // namespace blender::geometry
