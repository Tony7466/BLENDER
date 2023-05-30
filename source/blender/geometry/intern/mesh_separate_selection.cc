/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_multi_value_map.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.h"

#include "GEO_mesh_separate.hh"

namespace blender::geometry {

static void create_reverse_map(const IndexMask &mask, MutableSpan<int> r_map)
{
  mask.foreach_index_optimized<int>(
      GrainSize(4049), [&](const int src_i, const int dst_i) { r_map[src_i] = dst_i; });
}

static void remap_verts(const OffsetIndices<int> src_polys,
                        const OffsetIndices<int> dst_polys,
                        const int src_verts_num,
                        const IndexMask &vert_selection,
                        const IndexMask &edge_selection,
                        const IndexMask &poly_selection,
                        const Span<int2> src_edges,
                        const Span<int> src_corner_verts,
                        MutableSpan<int2> dst_edges,
                        MutableSpan<int> dst_corner_verts)
{
  Array<int> map(src_verts_num);
  create_reverse_map(vert_selection, map);
  threading::parallel_invoke(
      vert_selection.size() > 1024,
      [&]() {
        poly_selection.foreach_index(
            GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
              const IndexRange src_poly = src_polys[src_i];
              const IndexRange dst_poly = dst_polys[dst_i];
              for (const int i : src_poly.index_range()) {
                dst_corner_verts[dst_poly[i]] = map[src_corner_verts[src_poly[i]]];
              }
            });
      },
      [&]() {
        edge_selection.foreach_index(GrainSize(512),
                                     [&](const int64_t src_i, const int64_t dst_i) {
                                       dst_edges[dst_i][0] = map[src_edges[src_i][0]];
                                       dst_edges[dst_i][1] = map[src_edges[src_i][1]];
                                     });
      });
}

static void remap_edges(const OffsetIndices<int> src_polys,
                        const OffsetIndices<int> dst_polys,
                        const int src_edges_num,
                        const IndexMask &edge_selection,
                        const IndexMask &poly_selection,
                        const Span<int> src_corner_edges,
                        MutableSpan<int> dst_corner_edges)
{
  Array<int> map(src_edges_num);
  create_reverse_map(edge_selection, map);
  poly_selection.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    const IndexRange src_poly = src_polys[src_i];
    const IndexRange dst_poly = dst_polys[dst_i];
    for (const int i : src_poly.index_range()) {
      dst_corner_edges[dst_poly[i]] = map[src_corner_edges[src_poly[i]]];
    }
  });
}

/** A vertex is selected if it's used by a selected edge. */
static IndexMask vert_selection_from_edge(const Span<int2> edges,
                                          const IndexMask &edge_selection,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  Array<bool> span(verts_num, false);
  edge_selection.foreach_index_optimized<int>([&](const int i) {
    span[edges[i][0]] = true;
    span[edges[i][1]] = true;
  });
  return IndexMask::from_bools(span, memory);
}

static IndexMask poly_selection_from_mapped_corner(const OffsetIndices<int> polys,
                                                   const Span<int> corner_verts_or_edges,
                                                   const IndexMask &vert_or_edge_selection,
                                                   IndexMaskMemory &memory)
{
  // TODO: To bits first?
  return IndexMask::from_predicate(
      polys.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const Span<int> poly = corner_verts_or_edges.slice(polys[i]);
        return std::all_of(poly.begin(), poly.end(), [&](const int edge) {
          return vert_or_edge_selection.contains(edge);
        });
      });
}

/** A face is selected if all of its vertices are selected. */
static IndexMask poly_selection_from_vert(const OffsetIndices<int> polys,
                                          const Span<int> corner_verts,
                                          const IndexMask &vert_selection,
                                          IndexMaskMemory &memory)
{
  return poly_selection_from_mapped_corner(polys, corner_verts, vert_selection, memory);
}

/** A face is selected if all of its edges are selected. */
static IndexMask poly_selection_from_edge(const OffsetIndices<int> polys,
                                          const Span<int> corner_edges,
                                          const IndexMask &edge_selection,
                                          IndexMaskMemory &memory)
{
  return poly_selection_from_mapped_corner(polys, corner_edges, edge_selection, memory);
}

static IndexMask mapped_corner_selection_from_poly(const OffsetIndices<int> polys,
                                                   const IndexMask &poly_selection,
                                                   const Span<int> corner_verts_or_edges,
                                                   const int verts_or_edges_num,
                                                   IndexMaskMemory &memory)
{
  Array<bool> array(verts_or_edges_num);
  poly_selection.foreach_index(GrainSize(512), [&](const int64_t i) {
    const Span<int> poly_edges = corner_verts_or_edges.slice(polys[i]);
    array.as_mutable_span().fill_indices(poly_edges, true);
  });
  return IndexMask::from_bools(array, memory);
}

/** A vertex is selected if it is used by a selected face. */
static IndexMask vert_selection_from_poly(const OffsetIndices<int> polys,
                                          const IndexMask &poly_selection,
                                          const Span<int> corner_verts,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_poly(polys, poly_selection, corner_verts, verts_num, memory);
}

/** An edge is selected if it is used by a selected face. */
static IndexMask edge_selection_from_poly(const OffsetIndices<int> polys,
                                          const IndexMask &poly_selection,
                                          const Span<int> corner_edges,
                                          const int edges_num,
                                          IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_poly(polys, poly_selection, corner_edges, edges_num, memory);
}

Mesh *mesh_copy_selection(const Mesh &src_mesh,
                          const fn::Field<bool> &selection,
                          const eAttrDomain selection_domain,
                          const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_polys = src_mesh.polys();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();

  // TODO: Local
  IndexMaskMemory memory;
  IndexMask vert_selection;
  IndexMask edge_selection;
  IndexMask poly_selection;
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_POINT);
      fn::FieldEvaluator evaluator(context, src_mesh.totvert);
      evaluator.add(selection);
      evaluator.evaluate();
      vert_selection = evaluator.get_evaluated_as_mask(0);
      BitVector<> selected_verts(src_mesh.totvert);
      vert_selection.to_bits(selected_verts);
      threading::parallel_invoke(
          vert_selection.size() > 1024,
          [&]() {
            edge_selection = IndexMask::from_predicate(
                src_edges.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
                  const int2 edge = src_edges[i];
                  return selected_verts[edge[0]] && selected_verts[edge[1]];
                });
          },
          [&]() {
            poly_selection = poly_selection_from_vert(
                src_polys, src_corner_verts, vert_selection, memory);
          });
      break;
    }
    case ATTR_DOMAIN_EDGE: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_EDGE);
      fn::FieldEvaluator evaluator(context, src_mesh.totedge);
      evaluator.add(selection);
      evaluator.evaluate();
      edge_selection = evaluator.get_evaluated_as_mask(0);
      threading::parallel_invoke(
          edge_selection.size() > 1024,
          [&]() {
            vert_selection = vert_selection_from_edge(
                src_edges, edge_selection, src_mesh.totvert, memory);
          },
          [&]() {
            poly_selection = poly_selection_from_edge(
                src_polys, src_corner_edges, edge_selection, memory);
          });
      break;
    }
    case ATTR_DOMAIN_FACE: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_POINT);
      fn::FieldEvaluator evaluator(context, src_mesh.totpoly);
      evaluator.add(selection);
      evaluator.evaluate();
      poly_selection = evaluator.get_evaluated_as_mask(0);
      threading::parallel_invoke(
          poly_selection.size() > 1024,
          [&]() {
            vert_selection = vert_selection_from_poly(
                src_polys, poly_selection, src_corner_verts, src_mesh.totvert, memory);
          },
          [&]() {
            edge_selection = edge_selection_from_poly(
                src_polys, poly_selection, src_corner_edges, src_mesh.totedge, memory);
          });
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(
      vert_selection.size(), edge_selection.size(), poly_selection.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_selection, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  dst_attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  dst_attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());

  threading::parallel_invoke(
      vert_selection.size() > 1024,
      [&]() {
        remap_verts(src_polys,
                    dst_polys,
                    src_mesh.totvert,
                    vert_selection,
                    edge_selection,
                    poly_selection,
                    src_edges,
                    src_corner_verts,
                    dst_mesh->edges_for_write(),
                    dst_mesh->corner_verts_for_write());
      },
      [&]() {
        remap_edges(src_polys,
                    dst_polys,
                    src_edges.size(),
                    edge_selection,
                    poly_selection,
                    src_corner_edges,
                    dst_mesh->corner_edges_for_write());
      });

  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, vert_selection, dst_attributes);
  bke::gather_attributes(src_attributes,
                         ATTR_DOMAIN_EDGE,
                         propagation_info,
                         {".edge_verts"},
                         edge_selection,
                         dst_attributes);
  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_selection, dst_attributes);
  bke::gather_attributes_group_to_group(src_attributes,
                                        ATTR_DOMAIN_CORNER,
                                        propagation_info,
                                        {".corner_edge", ".corner_vert"},
                                        src_polys,
                                        dst_polys,
                                        poly_selection,
                                        dst_attributes);

  return dst_mesh;
}

Mesh *mesh_copy_selection_keep_verts(
    const Mesh &src_mesh,
    const fn::Field<bool> &selection,
    const eAttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_polys = src_mesh.polys();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();

  IndexMaskMemory memory;
  IndexMask edge_selection;
  IndexMask poly_selection;
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_POINT);
      fn::FieldEvaluator evaluator(context, src_mesh.totvert);
      evaluator.add(selection);
      evaluator.evaluate();
      const VArraySpan<bool> vert_selection = evaluator.get_evaluated<bool>(0);

      // TODO: Deduplicate
      edge_selection = IndexMask::from_predicate(
          src_edges.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
            const int2 edge = src_edges[i];
            return vert_selection[edge[0]] && vert_selection[edge[1]];
          });
      poly_selection = IndexMask::from_predicate(
          src_polys.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
            const Span<int> poly_verts = src_corner_verts.slice(src_polys[i]);
            return std::all_of(poly_verts.begin(), poly_verts.end(), [&](const int vert) {
              return vert_selection[vert];
            });
          });
      break;
    }
    case ATTR_DOMAIN_EDGE: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_EDGE);
      fn::FieldEvaluator evaluator(context, src_mesh.totedge);
      evaluator.add(selection);
      evaluator.evaluate();
      edge_selection = evaluator.get_evaluated_as_mask(0);
      poly_selection = poly_selection_from_edge(
          src_polys, src_corner_edges, edge_selection, memory);
      break;
    }
    case ATTR_DOMAIN_FACE: {
      const bke::MeshFieldContext context(src_mesh, ATTR_DOMAIN_FACE);
      fn::FieldEvaluator evaluator(context, src_polys.size());
      evaluator.add(selection);
      evaluator.evaluate();
      poly_selection = evaluator.get_evaluated_as_mask(0);
      edge_selection = edge_selection_from_poly(
          src_polys, poly_selection, src_corner_edges, src_edges.size(), memory);
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(
      src_mesh.totvert, edge_selection.size(), poly_selection.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  CustomData_free_layer_named(&dst_mesh->vdata, "position", 0);
  dst_mesh->totvert = src_mesh.totvert;

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_selection, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();

  dst_attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());

  remap_edges(src_polys,
              dst_polys,
              src_edges.size(),
              edge_selection,
              poly_selection,
              src_corner_edges,
              dst_mesh->corner_edges_for_write());

  bke::copy_attributes(src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, dst_attributes);
  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_EDGE, propagation_info, {}, edge_selection, dst_attributes);
  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_selection, dst_attributes);
  bke::gather_attributes_group_to_group(src_attributes,
                                        ATTR_DOMAIN_CORNER,
                                        propagation_info,
                                        {".corner_edge"},
                                        src_polys,
                                        dst_polys,
                                        poly_selection,
                                        dst_attributes);

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  return dst_mesh;
}

Mesh *mesh_copy_selection_keep_verts_edges(
    const Mesh &mesh,
    const fn::Field<bool> &selection,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const bke::MeshFieldContext context(mesh, ATTR_DOMAIN_FACE);
  fn::FieldEvaluator evaluator(context, mesh.totpoly);
  evaluator.set_selection(selection);
  evaluator.evaluate();
  const IndexMask poly_selection = evaluator.get_evaluated_selection_as_mask();
  if (poly_selection.is_empty()) {
    return nullptr;
  }
  if (poly_selection.size() == mesh.totpoly) {
    return BKE_mesh_copy_for_eval(&mesh);
  }

  Mesh *dst_mesh = BKE_mesh_new_nomain(0, 0, poly_selection.size(), 0);
  CustomData_free_layer_named(&dst_mesh->vdata, "position", 0);
  CustomData_free_layer_named(&dst_mesh->edata, ".edge_verts", 0);
  dst_mesh->totvert = mesh.totvert;
  dst_mesh->totedge = mesh.totedge;
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &mesh);

  const OffsetIndices src_polys = mesh.polys();

  const OffsetIndices<int> dst_polys = offset_indices::gather_selected_offsets(
      src_polys, poly_selection, dst_mesh->poly_offsets_for_write());
  dst_mesh->totloop = dst_polys.total_size();

  const bke::AttributeAccessor src_attributes = mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  bke::copy_attributes(src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, dst_attributes);
  bke::copy_attributes(src_attributes, ATTR_DOMAIN_EDGE, propagation_info, {}, dst_attributes);
  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_FACE, propagation_info, {}, poly_selection, dst_attributes);
  bke::gather_attributes_group_to_group(src_attributes,
                                        ATTR_DOMAIN_CORNER,
                                        propagation_info,
                                        {},
                                        src_polys,
                                        dst_polys,
                                        poly_selection,
                                        dst_attributes);

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = mesh.runtime->bounds_cache;
  return dst_mesh;
}

}  // namespace blender::geometry
