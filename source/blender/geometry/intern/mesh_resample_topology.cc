/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_offset_indices.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.h"

#include "GEO_mesh_resample_topology.hh"

namespace blender::detail {

class MeshMarket {
 private:
  struct CustomOffsets {
    std::optional<Array<int>> vert_offsets;
    std::optional<Array<int>> edge_offsets;
    std::optional<Array<int>> loop_offsets;
    std::optional<Array<int>> face_offsets;
  };

  struct CustomData {
    IndexRange vert_range;
    IndexRange edge_range;
    IndexRange loop_range;
    IndexRange face_range;

    CustomOffsets offsets;

    void debug_print(const char *pref = "") const
    {
      printf("%s: CustomData: v{%d : %d, %d}, e{%d : %d, %d}, l{%d : %d, %d}, f{%d : %d, %d};\n",
             pref,
             int(vert_range.start()),
             int(vert_range.size()),
             int(vert_range.one_after_last()),
             int(edge_range.start()),
             int(edge_range.size()),
             int(edge_range.one_after_last()),
             int(loop_range.start()),
             int(loop_range.size()),
             int(loop_range.one_after_last()),
             int(face_range.start()),
             int(face_range.size()),
             int(face_range.one_after_last()));
    };
  };

  Vector<CustomData> customs_;

 public:
  MeshMarket() = default;

  int add_custom(const int vert,
                 const int edge,
                 const int loop,
                 const int face,
                 const CustomOffsets offset_values = {})
  {
    if (customs_.is_empty()) {
      const CustomData new_range{IndexRange(vert),
                                 IndexRange(edge),
                                 IndexRange(loop),
                                 IndexRange(face),
                                 std::move(offset_values)};
      customs_.append(new_range);
      return 0;
    }

    const CustomData prev_range = customs_.last();

    const CustomData new_range{IndexRange(prev_range.vert_range.one_after_last(), vert),
                               IndexRange(prev_range.edge_range.one_after_last(), edge),
                               IndexRange(prev_range.loop_range.one_after_last(), loop),
                               IndexRange(prev_range.face_range.one_after_last(), face),
                               std::move(offset_values)};

    return customs_.append_and_get_index(new_range);
    ;
  }

  int add_custom(const VArray<int> verts,
                 const VArray<int> edges,
                 const VArray<int> loops,
                 const VArray<int> faces,
                 const bool save_offsets = true)
  {

    auto total_size = [save_offsets](const VArray<int> &counts,
                                     std::optional<Array<int>> &offsets) -> int {
      if (counts.is_single()) {
        return counts.get_internal_single() * counts.size();
      }
      else {
        Array<int> vert_accumulate(counts.size() + 1);
        vert_accumulate.last() = 0;
        counts.materialize(vert_accumulate.as_mutable_span().drop_back(1));
        offset_indices::accumulate_counts_to_offsets(vert_accumulate.as_mutable_span());
        const int total_size = vert_accumulate.last();
        if (save_offsets) {
          offsets.emplace(std::move(vert_accumulate));
        }

        return total_size;
      }
    };

    CustomOffsets offsets;

    const int vert_total_size = total_size(std::move(verts), offsets.vert_offsets);
    const int edge_total_size = total_size(std::move(edges), offsets.edge_offsets);
    const int loop_total_size = total_size(std::move(loops), offsets.loop_offsets);
    const int face_total_size = total_size(std::move(faces), offsets.face_offsets);

    return this->add_custom(
        vert_total_size, edge_total_size, loop_total_size, face_total_size, std::move(offsets));
  }

  Mesh *build_new_mesh()
  {
    const CustomData &last_custom = customs_.last();

    return BKE_mesh_new_nomain(last_custom.vert_range.one_after_last(),
                               last_custom.edge_range.one_after_last(),
                               0,
                               last_custom.loop_range.one_after_last(),
                               last_custom.face_range.one_after_last());
  }

  IndexRange get_vert_range_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    return last_custom.vert_range;
  }

  IndexRange get_edge_range_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    return last_custom.edge_range;
  }

  IndexRange get_loop_range_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    return last_custom.loop_range;
  }

  IndexRange get_face_range_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    return last_custom.face_range;
  }

  OffsetIndices<int> get_vert_offsets_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    BLI_assert(last_custom.offsets.vert_offsets);

    return OffsetIndices<int>(last_custom.offsets.vert_offsets->as_span());
  }

  OffsetIndices<int> get_edge_offsets_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    BLI_assert(last_custom.offsets.edge_offsets);

    return OffsetIndices<int>(last_custom.offsets.edge_offsets->as_span());
  }

  OffsetIndices<int> get_loop_offsets_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    BLI_assert(last_custom.offsets.loop_offsets);

    return OffsetIndices<int>(last_custom.offsets.loop_offsets->as_span());
  }

  OffsetIndices<int> get_face_offsets_in(const int custom_index) const
  {
    const CustomData &last_custom = customs_[custom_index];
    BLI_assert(last_custom.offsets.face_offsets);

    return OffsetIndices<int>(last_custom.offsets.face_offsets->as_span());
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
  return;
  for (const int poly_i : polys.index_range()) {
    const MPoly src_poly = polys[poly_i];
    const Span<MLoop> src_loops = loops.slice(src_poly.loopstart, src_poly.totloop);
    const IndexRange poly_vertices = offsets[poly_i];

    if (src_loops.size() == 4) {

      const int horizontal = math::max(resample_edge_num[src_loops[0].e],
                                       resample_edge_num[src_loops[2].e]);
      const int vertical = math::max(resample_edge_num[src_loops[1].e],
                                     resample_edge_num[src_loops[3].e]);

      const int vert_index_a = src_loops[0].v;
      const int vert_index_b = src_loops[1].v;
      const int vert_index_c = src_loops[2].v;
      const int vert_index_d = src_loops[3].v;

      for (const int horizontal_i : IndexRange(horizontal)) {
        for (const int vertical_i : IndexRange(vertical)) {
          const int offset_index = vertical * horizontal_i + vertical_i;

          const float horizontal_factor = float(horizontal_i + 1) / float(horizontal + 1);
          const float vertical_factor = float(vertical_i + 1) / float(vertical + 1);

          const T aa = attribute_math::mix2<T>(
              horizontal_factor, src[vert_index_a], src[vert_index_b]);
          const T bb = attribute_math::mix2<T>(
              horizontal_factor, src[vert_index_d], src[vert_index_c]);

          dst[poly_vertices[offset_index]] = attribute_math::mix2<T>(vertical_factor, aa, bb);
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
    const ResampleTopologyMode fill_mode,
    const Map<bke::AttributeIDRef, bke::AttributeKind> &attributes,
    const Mesh &src_mesh,
    Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

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

    switch (fill_mode) {
      case FILL_GRID: {
        using namespace propagation_for_grid;
        attribute_on_domain(attribute.domain,
                            mesh_builder,
                            resample_edge_num,
                            src_mesh,
                            src_attribute_value,
                            dst_mesh,
                            dst_attribute_value);
        break;
      }
      case FILL_NGONE: {
        using namespace propagation_for_ngone;
        attribute_on_domain(attribute.domain,
                            mesh_builder,
                            src_mesh,
                            src_attribute_value,
                            dst_mesh,
                            dst_attribute_value);
        break;
      }
      case FILL_DELONE: {
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }

    result_attribute.finish();
  }
}

void compute_new_mesh(const Mesh &mesh,
                      const Span<int> resample_edge_num,
                      const ResampleTopologyMode fill_mode,
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

  switch (fill_mode) {
    case FILL_GRID: {
      /* Face grid. */
      {
        const Span<MLoop> src_loops = mesh.loops();
        const Span<MPoly> src_polys = mesh.polys();

        VArray<int> new_verts_for_original_faces = VArray<int>::ForFunc(
            mesh.totpoly, [resample_edge_num, src_loops, src_polys](const int64_t index) -> int {
              const MPoly face = src_polys[index];
              const IndexRange loop_range(face.loopstart, face.totloop);
              const Span<MLoop> loops = src_loops.slice(loop_range);
              
              if (loops.size() == 3) {
                return 0;
              }else if (loops.size() == 4){
                const int &points_num_edge_a = resample_edge_num[loops[0].e];
                const int &points_num_edge_b = resample_edge_num[loops[1].e];
                const int &points_num_edge_c = resample_edge_num[loops[2].e];
                const int &points_num_edge_d = resample_edge_num[loops[3].e];
                
                const bool horizontal_equal = points_num_edge_a == points_num_edge_c;
                const bool vertical_equal = points_num_edge_b == points_num_edge_d;
                if (horizontal_equal && vertical_equal) {
                  return points_num_edge_a * points_num_edge_b;
                }

                const int horizontal_connections = math::max(points_num_edge_a, points_num_edge_c);
                const int vertical_connections = math::max(points_num_edge_b, points_num_edge_d);
printf("- horizontal_connections: %d, vertical_connections: %d;\n", horizontal_connections, vertical_connections);
                const int edges_num_edge_a = points_num_edge_a + 1;
                const int edges_num_edge_b = points_num_edge_b + 1;
                const int edges_num_edge_c = points_num_edge_c + 1;
                const int edges_num_edge_d = points_num_edge_d + 1;
                
                const int max_horizontal_edge_num = math::max<int>(edges_num_edge_a, edges_num_edge_c);
                const int min_horizontal_edge_num = math::min<int>(edges_num_edge_a, edges_num_edge_c);
                const int horizontal_corner_connected_edges = (min_horizontal_edge_num * 2) / max_horizontal_edge_num; // * 2 X!
                
printf("- max_horizontal_edge_num: %d, min_horizontal_edge_num: %d, horizontal_corner_connected_edges: %d;\n",
    max_horizontal_edge_num, min_horizontal_edge_num, horizontal_corner_connected_edges);
                //const bool left_righr_side_corners = points_num_edge_a < points_num_edge_c;
                
                const int max_vertical_edge_num = math::max<int>(edges_num_edge_b, edges_num_edge_d);
                const int min_vertical_edge_num = math::min<int>(edges_num_edge_b, edges_num_edge_d);
                const int vertical_corner_connected_edges = (min_vertical_edge_num * 2) / max_vertical_edge_num; // * 2 X!
                
printf("- max_vertical_edge_num: %d, min_vertical_edge_num: %d, vertical_corner_connected_edges: %d;\n",
    max_vertical_edge_num, min_vertical_edge_num, vertical_corner_connected_edges);
                //const bool top_down_side_corners = points_num_edge_b < points_num_edge_d;
                
                
                const int horizontal_body_count = horizontal_connections - horizontal_corner_connected_edges * 2;
                const int vertical_body_count = vertical_connections - vertical_corner_connected_edges * 2;
                
printf("- horizontal_body_count: %d, vertical_body_count: %d;\n",
    horizontal_body_count, vertical_body_count);
    
                const int result = horizontal_body_count * vertical_body_count +
                                   horizontal_body_count * vertical_corner_connected_edges +
                                   vertical_body_count * horizontal_corner_connected_edges +
                                   horizontal_connections * vertical_corner_connected_edges +
                                   vertical_connections * horizontal_corner_connected_edges;

                
printf("- result: %d;\n",
    result);
    
                return math::max(result, 0);
              }

              return 0;
            });

        r_mesh_builder.add_custom(std::move(new_verts_for_original_faces),
                                  VArray<int>::ForSingle(0, 0),
                                  VArray<int>::ForSingle(0, 0),
                                  VArray<int>::ForSingle(0, 0));
      }
      break;
    }
    case FILL_NGONE: {
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
      break;
    }
    case FILL_DELONE: {
      break;
    }
    default: {
      BLI_assert_unreachable();
    }
  }
}

void build_new_mesh_topology(const Mesh &mesh,
                             const Span<int> /*resample_edge_num*/,
                             const ResampleTopologyMode fill_mode,
                             const detail::MeshMarket &mesh_builder,
                             Mesh &r_new_mesh)
{
  build_edge_vert_indices(mesh_builder.get_edge_offsets_in(1),
                          mesh_builder.get_vert_offsets_in(1),
                          mesh.edges(),
                          mesh_builder.get_vert_range_in(1),
                          r_new_mesh.edges_for_write().slice(mesh_builder.get_edge_range_in(1)));

  switch (fill_mode) {
    case FILL_GRID: {
      break;
    }
    case FILL_NGONE: {
      using namespace ngone_fill;
      build_faces_loops(mesh.edges(),
                        mesh.loops(),
                        mesh_builder.get_edge_offsets_in(1),
                        mesh_builder.get_loop_offsets_in(2),
                        r_new_mesh.edges(),
                        r_new_mesh.loops_for_write());
      build_faces(mesh.polys(), mesh_builder.get_loop_offsets_in(2), r_new_mesh.polys_for_write());
      break;
    }
    case FILL_DELONE: {
      break;
    }
    default: {
      BLI_assert_unreachable();
    }
  }
}

Mesh *resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const ResampleTopologyMode fill_mode,
                        const Map<bke::AttributeIDRef, bke::AttributeKind> attributes)
{
  detail::MeshMarket mesh_builder;

  compute_new_mesh(mesh, resample_edge_num, fill_mode, mesh_builder);

  Mesh &result = *mesh_builder.build_new_mesh();

  build_new_mesh_topology(mesh, resample_edge_num, fill_mode, mesh_builder, result);

  propagate_attributes_on_new_mesh(
      mesh_builder, resample_edge_num, fill_mode, attributes, mesh, result);

  return &result;
}

}  // namespace blender::geometry
