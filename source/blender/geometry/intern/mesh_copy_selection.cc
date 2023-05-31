/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_copy_selection.hh"

namespace blender::geometry {

static void create_reverse_map(const IndexMask &mask, MutableSpan<int> r_map)
{
  mask.foreach_index_optimized<int>(
      GrainSize(4049), [&](const int src_i, const int dst_i) { r_map[src_i] = dst_i; });
}

static void remap_verts(const OffsetIndices<int> src_polys,
                        const OffsetIndices<int> dst_polys,
                        const int src_verts_num,
                        const IndexMask &vert_mask,
                        const IndexMask &edge_mask,
                        const IndexMask &poly_mask,
                        const Span<int2> src_edges,
                        const Span<int> src_corner_verts,
                        MutableSpan<int2> dst_edges,
                        MutableSpan<int> dst_corner_verts)
{
  Array<int> map(src_verts_num);
  create_reverse_map(vert_mask, map);
  threading::parallel_invoke(
      vert_mask.size() > 1024,
      [&]() {
        poly_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
          const IndexRange src_poly = src_polys[src_i];
          const IndexRange dst_poly = dst_polys[dst_i];
          for (const int i : src_poly.index_range()) {
            dst_corner_verts[dst_poly[i]] = map[src_corner_verts[src_poly[i]]];
          }
        });
      },
      [&]() {
        edge_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
          dst_edges[dst_i][0] = map[src_edges[src_i][0]];
          dst_edges[dst_i][1] = map[src_edges[src_i][1]];
        });
      });
}

static void remap_edges(const OffsetIndices<int> src_polys,
                        const OffsetIndices<int> dst_polys,
                        const int src_edges_num,
                        const IndexMask &edge_mask,
                        const IndexMask &poly_mask,
                        const Span<int> src_corner_edges,
                        MutableSpan<int> dst_corner_edges)
{
  Array<int> map(src_edges_num);
  create_reverse_map(edge_mask, map);
  poly_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    const IndexRange src_poly = src_polys[src_i];
    const IndexRange dst_poly = dst_polys[dst_i];
    for (const int i : src_poly.index_range()) {
      dst_corner_edges[dst_poly[i]] = map[src_corner_edges[src_poly[i]]];
    }
  });
}

/** A vertex is selected if it's used by a selected edge. */
static IndexMask vert_selection_from_edge(const Span<int2> edges,
                                          const IndexMask &edge_mask,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  Array<bool> span(verts_num, false);
  edge_mask.foreach_index_optimized<int>([&](const int i) {
    span[edges[i][0]] = true;
    span[edges[i][1]] = true;
  });
  return IndexMask::from_bools(span, memory);
}

static IndexMask mapped_corner_selection_from_poly(const OffsetIndices<int> polys,
                                                   const IndexMask &poly_mask,
                                                   const Span<int> corner_verts_or_edges,
                                                   const int verts_or_edges_num,
                                                   IndexMaskMemory &memory)
{
  Array<bool> array(verts_or_edges_num);
  poly_mask.foreach_index(GrainSize(512), [&](const int64_t i) {
    const Span<int> poly_edges = corner_verts_or_edges.slice(polys[i]);
    array.as_mutable_span().fill_indices(poly_edges, true);
  });
  return IndexMask::from_bools(array, memory);
}

/** A vertex is selected if it is used by a selected face. */
static IndexMask vert_selection_from_poly(const OffsetIndices<int> polys,
                                          const IndexMask &poly_mask,
                                          const Span<int> corner_verts,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_poly(polys, poly_mask, corner_verts, verts_num, memory);
}

/** An edge is selected if it is used by a selected face. */
static IndexMask edge_selection_from_poly(const OffsetIndices<int> polys,
                                          const IndexMask &poly_mask,
                                          const Span<int> corner_edges,
                                          const int edges_num,
                                          IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_poly(polys, poly_mask, corner_edges, edges_num, memory);
}

/** An edge is selected if both of its vertices are selected. */
static IndexMask edge_selection_from_vert(const Span<int2> edges,
                                          const Span<bool> vert_mask,
                                          IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      edges.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const int2 edge = edges[i];
        return vert_mask[edge[0]] && vert_mask[edge[1]];
      });
}

static IndexMask poly_selection_from_mapped_corner(const OffsetIndices<int> polys,
                                                   const Span<int> corner_verts_or_edges,
                                                   const Span<bool> vert_or_edge_selection,
                                                   IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      polys.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const Span<int> poly = corner_verts_or_edges.slice(polys[i]);
        return std::all_of(
            poly.begin(), poly.end(), [&](const int i) { return vert_or_edge_selection[i]; });
      });
}

/** A face is selected if all of its vertices are selected. */
static IndexMask poly_selection_from_vert(const OffsetIndices<int> polys,
                                          const Span<int> corner_verts,
                                          const Span<bool> vert_mask,
                                          IndexMaskMemory &memory)
{
  return poly_selection_from_mapped_corner(polys, corner_verts, vert_mask, memory);
}

/** A face is selected if all of its edges are selected. */
static IndexMask poly_selection_from_edge(const OffsetIndices<int> polys,
                                          const Span<int> corner_edges,
                                          const Span<bool> edge_mask,
                                          IndexMaskMemory &memory)
{
  return poly_selection_from_mapped_corner(polys, corner_edges, edge_mask, memory);
}

Mesh *mesh_copy_selection(const Mesh &src_mesh,
                          const fn::Field<bool> &selection_field,
                          const eAttrDomain selection_domain,
                          const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_polys = src_mesh.polys();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  const bke::MeshFieldContext context(src_mesh, selection_domain);
  fn::FieldEvaluator evaluator(context, src_attributes.domain_size(selection_domain));
  evaluator.add(selection_field);
  evaluator.evaluate();
  const VArray<bool> selection = evaluator.get_evaluated<bool>(0);
  if (const std::optional<bool> single = selection.get_if_single()) {
    return *single ? BKE_mesh_copy_for_eval(&src_mesh) : nullptr;
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask vert_mask;
  IndexMask edge_mask;
  IndexMask poly_mask;
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_mesh.totvert > 1024,
          [&]() { vert_mask = IndexMask::from_bools(span, memory.local()); },
          [&]() { edge_mask = edge_selection_from_vert(src_edges, span, memory.local()); },
          [&]() {
            poly_mask = poly_selection_from_vert(
                src_polys, src_corner_verts, span, memory.local());
          });
      break;
    }
    case ATTR_DOMAIN_EDGE: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() {
            edge_mask = IndexMask::from_bools(span, memory.local());
            vert_mask = vert_selection_from_edge(
                src_edges, edge_mask, src_mesh.totvert, memory.local());
          },
          [&]() {
            poly_mask = poly_selection_from_edge(
                src_polys, src_corner_edges, span, memory.local());
          });
      break;
    }
    case ATTR_DOMAIN_FACE: {
      const VArraySpan<bool> span(selection);
      poly_mask = IndexMask::from_bools(span, memory.local());
      threading::parallel_invoke(
          poly_mask.size() > 1024,
          [&]() {
            vert_mask = vert_selection_from_poly(
                src_polys, poly_mask, src_corner_verts, src_mesh.totvert, memory.local());
          },
          [&]() {
            edge_mask = edge_selection_from_poly(
                src_polys, poly_mask, src_corner_edges, src_mesh.totedge, memory.local());
          });
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  if (vert_mask.is_empty()) {
    return nullptr;
  }
  if (vert_mask.size() == src_mesh.totvert) {
    return BKE_mesh_copy_for_eval(&src_mesh);
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(vert_mask.size(), edge_mask.size(), poly_mask.size(), 0);
  CustomData_free_layer_named(&dst_mesh->ldata, ".corner_vert", 0);
  CustomData_free_layer_named(&dst_mesh->ldata, ".corner_edge", 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_mask, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();
  dst_attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  dst_attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  threading::parallel_invoke(
      vert_mask.size() > 1024,
      [&]() {
        remap_verts(src_polys,
                    dst_polys,
                    src_mesh.totvert,
                    vert_mask,
                    edge_mask,
                    poly_mask,
                    src_edges,
                    src_corner_verts,
                    dst_edges,
                    dst_corner_verts);
      },
      [&]() {
        remap_edges(src_polys,
                    dst_polys,
                    src_edges.size(),
                    edge_mask,
                    poly_mask,
                    src_corner_edges,
                    dst_corner_edges);
      },
      [&]() {
        bke::gather_attributes(
            src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, vert_mask, dst_attributes);
        bke::gather_attributes(src_attributes,
                               ATTR_DOMAIN_EDGE,
                               propagation_info,
                               {".edge_verts"},
                               edge_mask,
                               dst_attributes);
        bke::gather_attributes(
            src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_mask, dst_attributes);
        bke::gather_attributes_group_to_group(src_attributes,
                                              ATTR_DOMAIN_CORNER,
                                              propagation_info,
                                              {".corner_edge", ".corner_vert"},
                                              src_polys,
                                              dst_polys,
                                              poly_mask,
                                              dst_attributes);
      });

  return dst_mesh;
}

Mesh *mesh_copy_selection_keep_verts(
    const Mesh &src_mesh,
    const fn::Field<bool> &selection_field,
    const eAttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_polys = src_mesh.polys();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  const bke::MeshFieldContext context(src_mesh, selection_domain);
  fn::FieldEvaluator evaluator(context, src_attributes.domain_size(selection_domain));
  evaluator.add(selection_field);
  evaluator.evaluate();
  const VArray<bool> selection = evaluator.get_evaluated<bool>(0);
  if (const std::optional<bool> single = selection.get_if_single()) {
    return *single ? BKE_mesh_copy_for_eval(&src_mesh) : nullptr;
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask edge_mask;
  IndexMask poly_mask;
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() { edge_mask = edge_selection_from_vert(src_edges, span, memory.local()); },
          [&]() {
            poly_mask = poly_selection_from_vert(
                src_polys, src_corner_verts, span, memory.local());
          });
      break;
    }
    case ATTR_DOMAIN_EDGE: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() { edge_mask = IndexMask::from_bools(span, memory.local()); },
          [&]() {
            poly_mask = poly_selection_from_edge(
                src_polys, src_corner_edges, span, memory.local());
          });
      break;
    }
    case ATTR_DOMAIN_FACE: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() {
            edge_mask = edge_selection_from_poly(
                src_polys, poly_mask, src_corner_edges, src_edges.size(), memory.local());
          },
          [&]() { poly_mask = IndexMask::from_bools(span, memory.local()); });
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  if (edge_mask.is_empty()) {
    return nullptr;
  }
  if (edge_mask.size() == src_mesh.totedge) {
    return BKE_mesh_copy_for_eval(&src_mesh);
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(0, edge_mask.size(), poly_mask.size(), 0);
  CustomData_free_layer_named(&dst_mesh->vdata, "position", 0);
  CustomData_free_layer_named(&dst_mesh->ldata, ".corner_vert", 0);
  CustomData_free_layer_named(&dst_mesh->ldata, ".corner_edge", 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  dst_mesh->totvert = src_mesh.totvert;
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_mask, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();
  dst_attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  threading::parallel_invoke(
      [&]() {
        remap_edges(src_polys,
                    dst_polys,
                    src_edges.size(),
                    edge_mask,
                    poly_mask,
                    src_corner_edges,
                    dst_corner_edges);
      },
      [&]() {
        bke::copy_attributes(
            src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, dst_attributes);
        bke::gather_attributes(
            src_attributes, ATTR_DOMAIN_EDGE, propagation_info, {}, edge_mask, dst_attributes);
        bke::gather_attributes(
            src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_mask, dst_attributes);
        bke::gather_attributes_group_to_group(src_attributes,
                                              ATTR_DOMAIN_CORNER,
                                              propagation_info,
                                              {".corner_edge"},
                                              src_polys,
                                              dst_polys,
                                              poly_mask,
                                              dst_attributes);
      });

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  return dst_mesh;
}

Mesh *mesh_copy_selection_keep_edges(
    const Mesh &src_mesh,
    const fn::Field<bool> &selection_field,
    const eAttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const OffsetIndices src_polys = src_mesh.polys();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  const bke::MeshFieldContext context(src_mesh, selection_domain);
  fn::FieldEvaluator evaluator(context, src_attributes.domain_size(selection_domain));
  evaluator.add(selection_field);
  const VArray<bool> selection = evaluator.get_evaluated<bool>(0);
  if (const std::optional<bool> single = selection.get_if_single()) {
    return *single ? BKE_mesh_copy_for_eval(&src_mesh) : nullptr;
  }

  IndexMaskMemory memory;
  IndexMask poly_mask;
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT:
      poly_mask = poly_selection_from_vert(
          src_polys, src_mesh.corner_verts(), VArraySpan(selection), memory);
      break;
    case ATTR_DOMAIN_EDGE:
      poly_mask = poly_selection_from_edge(
          src_polys, src_mesh.corner_edges(), VArraySpan(selection), memory);
      break;
    case ATTR_DOMAIN_FACE:
      poly_mask = IndexMask::from_bools(evaluator.get_evaluated<bool>(0), memory);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

  if (poly_mask.is_empty()) {
    return nullptr;
  }
  if (poly_mask.size() == src_mesh.totpoly) {
    return BKE_mesh_copy_for_eval(&src_mesh);
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(0, 0, poly_mask.size(), 0);
  CustomData_free_layer_named(&dst_mesh->vdata, "position", 0);
  CustomData_free_layer_named(&dst_mesh->edata, ".edge_verts", 0);
  dst_mesh->totvert = src_mesh.totvert;
  dst_mesh->totedge = src_mesh.totedge;
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_mask, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();

  bke::copy_attributes(src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, dst_attributes);
  bke::copy_attributes(src_attributes, ATTR_DOMAIN_EDGE, propagation_info, {}, dst_attributes);
  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_mask, dst_attributes);
  bke::gather_attributes_group_to_group(src_attributes,
                                        ATTR_DOMAIN_CORNER,
                                        propagation_info,
                                        {},
                                        src_polys,
                                        dst_polys,
                                        poly_mask,
                                        dst_attributes);

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  return dst_mesh;
}

}  // namespace blender::geometry
