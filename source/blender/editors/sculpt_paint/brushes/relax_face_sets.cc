/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"

#include "BKE_subdiv_ccg.hh"

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_base.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"

#include "editors/sculpt_paint/mesh_brush_common.hh"
#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

inline namespace relax_face_sets_cc {

struct MeshLocalData {
  Vector<float> factors;
  Vector<float> distances;
  Vector<Vector<int>> vert_neighbors;
  Vector<float3> translations;
};

struct GridLocalData {
  Vector<float3> positions;
  Vector<float3> new_positions;
  Vector<float> factors;
  Vector<float> distances;
  Vector<SubdivCCGNeighbors> vert_neighbors;
  Vector<float3> translations;
};

static std::array<float, 4> iteration_strengths(const float strength, const int stroke_iteration)
{
  if (stroke_iteration % 3 == 0) {
    return {strength, strength, strength, strength};
  }

  /* This operations needs a strength tweak as the relax deformation is too weak by default. */
  const float modified_strength = strength * 1.5f;
  return {modified_strength, modified_strength, strength, strength};
}

static void filter_factors_on_face_sets_mesh(const GroupedSpan<int> vert_to_face_map,
                                             const int *face_sets,
                                             const bool relax_face_sets,
                                             const Span<int> verts,
                                             const MutableSpan<float> factors)
{
  BLI_assert(verts.size() == factors.size());

  for (const int i : verts.index_range()) {
    if (relax_face_sets ==
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, verts[i]))
    {
      factors[i] = 0.0f;
    }
  }
}
static void filter_factors_on_face_sets_grids(const GroupedSpan<int> vert_to_face_map,
                                              const Span<int> corner_verts,
                                              const OffsetIndices<int> faces,
                                              const SubdivCCG &subdiv_ccg,
                                              const int *face_sets,
                                              const bool relax_face_sets,
                                              const Span<int> grids,
                                              const MutableSpan<float> factors)
{
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  BLI_assert(grids.size() * key.grid_area == factors.size());

  for (const int i : grids.index_range()) {
    const int start = i * key.grid_area;
    for (const int y : IndexRange(key.grid_size)) {
      for (const int x : IndexRange(key.grid_size)) {
        const int offset = CCG_grid_xy_to_index(key.grid_size, x, y);
        SubdivCCGCoord coord{};
        coord.grid_index = grids[i];
        coord.x = x;
        coord.y = y;
        if (relax_face_sets ==
            face_set::vert_has_unique_face_set_grids(
                vert_to_face_map, corner_verts, faces, face_sets, subdiv_ccg, coord))
        {
          factors[start + offset] = 0.0f;
        }
      }
    }
  }
}
static void filter_factors_on_face_sets_bmesh(const GroupedSpan<int> vert_to_face_map,
                                              const int *face_sets,
                                              const bool relax_face_sets,
                                              const Span<int> verts,
                                              const MutableSpan<float> factors)
{
  BLI_assert(verts.size() == factors.size());

  for (const int i : verts.index_range()) {
    if (relax_face_sets ==
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, verts[i]))
    {
      factors[i] = 0.0f;
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name Relax Vertex
 * \{ */

static Vector<int, 16> filtered_neighbors(const Span<int> neighbors,
                                          const bool filter_boundary_face_sets,
                                          FunctionRef<bool(int)> is_unique_element_fn,
                                          FunctionRef<bool(int)> is_boundary_element_fn)
{
  Vector<int, 16> result;
  for (const int vert : neighbors) {
    /* If we are filtering face sets, then we only want to affect vertices that have more than one
     * face set, i.e. are on the boundary of a face set and another face set. */
    if (filter_boundary_face_sets && is_unique_element_fn(vert)) {
      continue;
    }

    /* When the vertex to relax is boundary, use only connected boundary vertices for the average
     * position. */
    if (is_boundary_element_fn && is_boundary_element_fn(vert)) {
      continue;
    }

    result.append(vert);
  }
  return result;
}

static bool get_normal_boundary(const float3 &current_position,
                                const Span<float3> vert_positions,
                                const Span<int> neighbors,
                                float3 &r_new_normal)
{
  /* If we are not dealing with a corner vertex, skip this step.*/
  if (neighbors.size() != 2) {
    return false;
  }

  float3 normal(0.0f, 0.0f, 0.0f);
  for (const int vert : neighbors) {
    const float3 to_neighbor = vert_positions[vert] - current_position;
    normal += math::normalize(to_neighbor);
  }

  r_new_normal = math::normalize(normal);

  return true;
}

static bool get_average_position(const Span<float3> vert_positions,
                                 const Span<int> neighbors,
                                 float3 &r_new_position)
{
  if (neighbors.size() == 0) {
    return false;
  }

  float3 average_position(0.0f, 0.0f, 0.0f);
  for (const int vert : neighbors) {
    average_position += vert_positions[vert];
  }

  average_position *= math::rcp(float(neighbors.size()));
  r_new_position = average_position;

  return true;
}

static Vector<SubdivCCGCoord, 16> filtered_neighbors(
    const Span<SubdivCCGCoord> neighbors,
    const bool filter_boundary_face_sets,
    FunctionRef<bool(SubdivCCGCoord)> is_unique_element_fn,
    FunctionRef<bool(SubdivCCGCoord)> is_boundary_element_fn)
{
  Vector<SubdivCCGCoord, 16> result;
  for (const SubdivCCGCoord coord : neighbors) {
    /* If we are filtering face sets, then we only want to affect vertices that have more than one
     * face set, i.e. are on the boundary of a face set and another face set. */
    if (filter_boundary_face_sets && is_unique_element_fn(coord)) {
      continue;
    }

    /* When the vertex to relax is boundary, use only connected boundary vertices for the average
     * position. */
    if (is_boundary_element_fn && is_boundary_element_fn(coord)) {
      continue;
    }

    result.append(coord);
  }
  return result;
}

static bool get_normal_boundary(const CCGKey &key,
                                const Span<CCGElem *> elems,
                                const float3 &current_position,
                                const Span<SubdivCCGCoord> neighbors,
                                float3 &r_new_normal)
{
  /* If we are not dealing with a corner vertex, skip this step.*/
  if (neighbors.size() != 2) {
    return false;
  }

  float3 normal(0.0f, 0.0f, 0.0f);
  for (const SubdivCCGCoord &coord : neighbors) {
    const float3 to_neighbor = CCG_grid_elem_co(key, elems[coord.grid_index], coord.x, coord.y) -
                               current_position;
    normal += math::normalize(to_neighbor);
  }

  r_new_normal = math::normalize(normal);

  return true;
}

static bool get_average_position(const CCGKey &key,
                                 const Span<CCGElem *> elems,
                                 const Span<SubdivCCGCoord> neighbors,
                                 float3 &r_new_position)
{
  if (neighbors.size() == 0) {
    return false;
  }

  float3 average_position(0.0f, 0.0f, 0.0f);
  for (const SubdivCCGCoord &coord : neighbors) {

    average_position += CCG_grid_elem_co(key, elems[coord.grid_index], coord.x, coord.y);
  }

  average_position *= math::rcp(float(neighbors.size()));
  r_new_position = average_position;

  return true;
}

/** \} */

BLI_NOINLINE static void calc_relaxed_positions_faces(const OffsetIndices<int> faces,
                                                      const Span<int> corner_verts,
                                                      const int *face_sets,
                                                      const GroupedSpan<int> vert_to_face_map,
                                                      const BitSpan boundary_verts,
                                                      const Span<bool> hide_poly,
                                                      const Span<int> verts,
                                                      const Span<float3> vert_positions,
                                                      const Span<float3> vert_normals,
                                                      const bool relax_face_sets,
                                                      MeshLocalData &tls,
                                                      const MutableSpan<float3> new_positions)
{
  BLI_assert(verts.size() == new_positions.size());

  tls.vert_neighbors.reinitialize(verts.size());
  calc_vert_neighbors_interior(
      faces, corner_verts, vert_to_face_map, boundary_verts, hide_poly, verts, tls.vert_neighbors);
  const Span<Vector<int>> vert_neighbors = tls.vert_neighbors;

  for (const int i : verts.index_range()) {
    /* Don't modify corner vertices */
    if (vert_neighbors[i].size() <= 2) {
      new_positions[i] = vert_positions[verts[i]];
      continue;
    }

    Vector<int, 16> neighbors;
    if (boundary_verts[verts[i]]) {
      neighbors = filtered_neighbors(
          vert_neighbors[i],
          relax_face_sets,
          [&](const int vert) {
            return face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, vert);
          },
          [&](const int vert) { return !boundary_verts[vert]; });
    }
    else {
      neighbors = filtered_neighbors(vert_neighbors[i],
                                     relax_face_sets,
                                     [&](const int vert) {
                                       return face_set::vert_has_unique_face_set_mesh(
                                           vert_to_face_map, face_sets, vert);
                                     },
                                     {});
    }

    /* Smoothed position calculation */
    float3 smoothed_position;
    const bool has_new_position = get_average_position(
        vert_positions, neighbors, smoothed_position);

    if (!has_new_position) {
      new_positions[i] = vert_positions[verts[i]];
      continue;
    }

    /* Normal Calculation */
    float3 normal;
    if (boundary_verts[verts[i]]) {
      bool has_boundary_normal = get_normal_boundary(
          vert_positions[verts[i]], vert_positions, neighbors, normal);

      if (!has_boundary_normal) {
        normal = vert_normals[verts[i]];
      }
    }
    else {
      normal = vert_normals[verts[i]];
    }

    if (math::is_zero(normal)) {
      new_positions[i] = vert_positions[verts[i]];
      continue;
    }

    float4 plane;
    plane_from_point_normal_v3(plane, vert_positions[verts[i]], normal);

    float3 smooth_closest_plane;
    closest_to_plane_v3(smooth_closest_plane, plane, smoothed_position);

    float3 displacement = smooth_closest_plane - vert_positions[verts[i]];
    new_positions[i] = vert_positions[verts[i]] + displacement;
  }
}

BLI_NOINLINE static void apply_positions_faces(const Sculpt &sd,
                                               const Brush &brush,
                                               const Span<float3> positions_eval,
                                               const Span<float3> vert_normals,
                                               const PBVHNode &node,
                                               const float strength,
                                               const bool relax_face_sets,
                                               Object &object,
                                               MeshLocalData &tls,
                                               const Span<float3> new_positions,
                                               const MutableSpan<float3> positions_orig)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const Mesh &mesh = *static_cast<Mesh *>(object.data);

  const Span<int> verts = bke::pbvh::node_unique_verts(node);
  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;

  fill_factor_from_hide_and_mask(mesh, verts, factors);
  filter_region_clip_factors(ss, positions_eval, verts, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_brush_distances(
      ss, positions_eval, verts, eBrushFalloffShape(brush.falloff_shape), distances);
  filter_distances_with_radius(cache.radius, distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    auto_mask::calc_vert_factors(object, *cache.automasking, node, verts, factors);
  }

  scale_factors(factors, strength);

  calc_brush_texture_factors(ss, brush, positions_eval, verts, factors);

  filter_factors_on_face_sets_mesh(
      ss.vert_to_face_map, ss.face_sets, relax_face_sets, verts, factors);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;
  translations_from_new_positions(new_positions, verts, positions_eval, translations);
  scale_translations(translations, factors);

  write_translations(sd, object, positions_eval, verts, translations, positions_orig);
}

static void do_relax_face_sets_brush_mesh(const Sculpt &sd,
                                          const Brush &brush,
                                          Object &object,
                                          const Span<PBVHNode *> nodes,
                                          const float strength,
                                          const float relax_face_sets)
{
  const SculptSession &ss = *object.sculpt;
  Mesh &mesh = *static_cast<Mesh *>(object.data);
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  const PBVH &pbvh = *ss.pbvh;

  const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);
  const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);
  MutableSpan<float3> positions_orig = mesh.vert_positions_for_write();

  Array<int> node_offset_data;
  const OffsetIndices<int> node_vert_offsets = create_node_vert_offsets(nodes, node_offset_data);

  Array<float3> new_positions(node_vert_offsets.total_size());

  threading::EnumerableThreadSpecific<MeshLocalData> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    MeshLocalData &tls = all_tls.local();
    for (const int i : range) {
      calc_relaxed_positions_faces(faces,
                                   corner_verts,
                                   ss.face_sets,
                                   ss.vert_to_face_map,
                                   ss.vertex_info.boundary,
                                   hide_poly,
                                   bke::pbvh::node_unique_verts(*nodes[i]),
                                   positions_eval,
                                   vert_normals,
                                   relax_face_sets,
                                   tls,
                                   new_positions.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    MeshLocalData &tls = all_tls.local();
    for (const int i : range) {
      apply_positions_faces(sd,
                            brush,
                            positions_eval,
                            vert_normals,
                            *nodes[i],
                            strength,
                            relax_face_sets,
                            object,
                            tls,
                            new_positions.as_span().slice(node_vert_offsets[i]),
                            positions_orig);
    }
  });
}

BLI_NOINLINE static void calc_relaxed_positions_grids(const OffsetIndices<int> faces,
                                                      const Span<int> corner_verts,
                                                      const int *face_sets,
                                                      const GroupedSpan<int> vert_to_face_map,
                                                      const BitSpan boundary_verts,
                                                      const PBVHNode &node,
                                                      const bool relax_face_sets,
                                                      Object &object,
                                                      GridLocalData &tls,
                                                      const MutableSpan<float3> new_positions)
{
  SculptSession &ss = *object.sculpt;
  SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const Span<CCGElem *> elems = subdiv_ccg.grids;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

  const Span<int> grids = bke::pbvh::node_grid_indices(node);
  const int grid_verts_num = grids.size() * key.grid_area;
  BLI_assert(grid_verts_num == new_positions.size());

  tls.vert_neighbors.reinitialize(grid_verts_num);
  calc_vert_neighbors_interior(
      faces, corner_verts, boundary_verts, subdiv_ccg, grids, tls.vert_neighbors);
  const Span<SubdivCCGNeighbors> vert_neighbors = tls.vert_neighbors;

  tls.positions.reinitialize(grid_verts_num);
  const MutableSpan<float3> positions = tls.positions;
  gather_grids_positions(subdiv_ccg, grids, positions);
  for (const int i : grids.index_range()) {
    CCGElem *elem = elems[grids[i]];
    const int start = i * key.grid_area;
    for (const int y : IndexRange(key.grid_size)) {
      for (const int x : IndexRange(key.grid_size)) {
        const int offset = CCG_grid_xy_to_index(key.grid_size, x, y);
        const int grid_idx = start + offset;
        SubdivCCGCoord coord{};
        coord.grid_index = grids[i];
        coord.x = x;
        coord.y = y;

        bool is_boundary = BKE_subdiv_ccg_coord_is_mesh_boundary(
            faces, corner_verts, boundary_verts, subdiv_ccg, coord);

        const SubdivCCGNeighbors vert_neighbor = vert_neighbors[start + offset];
        /* Don't modify corner vertices */
        if (vert_neighbor.coords.size() <= 2) {
          new_positions[grid_idx] = positions[grid_idx];
          continue;
        }

        Vector<SubdivCCGCoord, 16> neighbors;
        if (is_boundary) {
          neighbors = filtered_neighbors(
              vert_neighbor.coords,
              relax_face_sets,
              [&](const SubdivCCGCoord &neighbor) {
                return face_set::vert_has_unique_face_set_grids(
                    vert_to_face_map, corner_verts, faces, face_sets, subdiv_ccg, neighbor);
              },
              [&](const SubdivCCGCoord &neighbor) {
                return !BKE_subdiv_ccg_coord_is_mesh_boundary(
                    faces, corner_verts, boundary_verts, subdiv_ccg, neighbor);
              });
        }
        else {
          neighbors = filtered_neighbors(
              vert_neighbor.coords,
              relax_face_sets,
              [&](const SubdivCCGCoord &neighbor) {
                return face_set::vert_has_unique_face_set_grids(
                    vert_to_face_map, corner_verts, faces, face_sets, subdiv_ccg, neighbor);
              },
              {});
        }

        /* Smoothed position calculation */
        float3 smoothed_position;
        const bool has_new_position = get_average_position(
            key, elems, neighbors, smoothed_position);

        if (!has_new_position) {
          new_positions[grid_idx] = positions[grid_idx];
          continue;
        }

        /* Normal Calculation */
        float3 normal;
        if (is_boundary) {
          bool has_boundary_normal = get_normal_boundary(
              key, elems, positions[grid_idx], neighbors, normal);

          if (!has_boundary_normal) {
            normal = CCG_elem_offset_no(key, elem, offset);
          }
        }
        else {
          normal = CCG_elem_offset_no(key, elem, offset);
        }

        if (math::is_zero(normal)) {
          new_positions[grid_idx] = positions[grid_idx];
          continue;
        }

        float4 plane;
        plane_from_point_normal_v3(plane, positions[grid_idx], normal);

        float3 smooth_closest_plane;
        closest_to_plane_v3(smooth_closest_plane, plane, smoothed_position);

        float3 displacement = smooth_closest_plane - positions[grid_idx];
        new_positions[grid_idx] = positions[grid_idx] + displacement;
      }
    }
  }
}

BLI_NOINLINE static void apply_positions_grids(const Sculpt &sd,
                                               const Brush &brush,
                                               const Span<int> corner_verts,
                                               const OffsetIndices<int> faces,
                                               const PBVHNode &node,
                                               const float strength,
                                               const bool relax_face_sets,
                                               Object &object,
                                               GridLocalData &tls,
                                               const Span<float3> new_positions)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

  const Span<int> grids = bke::pbvh::node_grid_indices(node);
  const int grid_verts_num = grids.size() * key.grid_area;

  tls.positions.reinitialize(grid_verts_num);
  const MutableSpan<float3> positions = tls.positions;
  gather_grids_positions(subdiv_ccg, grids, positions);

  tls.factors.reinitialize(grid_verts_num);
  const MutableSpan<float> factors = tls.factors;
  fill_factor_from_hide_and_mask(subdiv_ccg, grids, factors);
  filter_region_clip_factors(ss, positions, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, subdiv_ccg, grids, factors);
  }

  tls.distances.reinitialize(grid_verts_num);
  const MutableSpan<float> distances = tls.distances;
  calc_brush_distances(ss, positions, eBrushFalloffShape(brush.falloff_shape), distances);
  filter_distances_with_radius(cache.radius, distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    auto_mask::calc_grids_factors(object, *cache.automasking, node, grids, factors);
  }

  scale_factors(factors, strength);

  calc_brush_texture_factors(ss, brush, positions, factors);

  filter_factors_on_face_sets_grids(ss.vert_to_face_map,
                                    corner_verts,
                                    faces,
                                    subdiv_ccg,
                                    ss.face_sets,
                                    relax_face_sets,
                                    grids,
                                    factors);

  tls.translations.reinitialize(grid_verts_num);
  const MutableSpan<float3> translations = tls.translations;
  translations_from_new_positions(new_positions, positions, translations);
  scale_translations(translations, factors);

  clip_and_lock_translations(sd, ss, positions, translations);
  apply_translations(translations, grids, subdiv_ccg);
}

static void do_relax_face_sets_brush_grids(const Sculpt &sd,
                                           const Brush &brush,
                                           Object &object,
                                           const Span<PBVHNode *> nodes,
                                           const float strength,
                                           const float relax_face_sets)
{
  const SculptSession &ss = *object.sculpt;
  SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

  Mesh &mesh = *static_cast<Mesh *>(object.data);
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  Array<int> node_offset_data;
  const OffsetIndices<int> node_vert_offsets = create_node_vert_offsets(
      nodes, key, node_offset_data);

  Array<float3> new_positions(node_vert_offsets.total_size());

  threading::EnumerableThreadSpecific<GridLocalData> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    GridLocalData &tls = all_tls.local();
    for (const int i : range) {
      calc_relaxed_positions_grids(faces,
                                   corner_verts,
                                   ss.face_sets,
                                   ss.vert_to_face_map,
                                   ss.vertex_info.boundary,
                                   *nodes[i],
                                   relax_face_sets,
                                   object,
                                   tls,
                                   new_positions.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    GridLocalData &tls = all_tls.local();
    for (const int i : range) {
      apply_positions_grids(sd,
                            brush,
                            corner_verts,
                            faces,
                            *nodes[i],
                            strength,
                            relax_face_sets,
                            object,
                            tls,
                            new_positions.as_span().slice(node_vert_offsets[i]));
    }
  });
}

}  // namespace relax_face_sets_cc

void do_relax_face_sets_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);

  SCULPT_boundary_info_ensure(object);

  const SculptSession &ss = *object.sculpt;
  const std::array<float, 4> strengths = iteration_strengths(ss.cache->bstrength,
                                                             ss.cache->iteration_count);

  /* On every third step of the stroke, behave more similarly to the Topology Relax brush */
  const bool relax_face_sets = !(ss.cache->iteration_count % 3 == 0);

  for (const float strength : strengths) {
    switch (BKE_pbvh_type(*ss.pbvh)) {
      case PBVH_FACES:
        do_relax_face_sets_brush_mesh(
            sd, brush, object, nodes, strength * strength, relax_face_sets);
        break;
      case PBVH_GRIDS:
        do_relax_face_sets_brush_grids(
            sd, brush, object, nodes, strength * strength, relax_face_sets);
        break;
      case PBVH_BMESH:
        break;
    }
  }
}
}  // namespace blender::ed::sculpt_paint
