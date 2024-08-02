/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#include "BKE_brush.hh"
#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"

#include "BKE_mesh.hh"
#include "BKE_pbvh.hh"
#include "BKE_subdiv_ccg.hh"
#include "BLI_array_utils.hh"

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_rotation_legacy.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"

#include "editors/sculpt_paint/mesh_brush_common.hh"
#include "editors/sculpt_paint/paint_intern.hh"
#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {
#define BOUNDARY_VERTEX_NONE -1
#define BOUNDARY_STEPS_NONE -1

inline namespace boundary_cc {

static void filter_uninitialized_verts(const Span<int> propagation_steps,
                                       const MutableSpan<float> factors)
{
  BLI_assert(propagation_steps.size() == factors.size());

  for (const int i : factors.index_range()) {
    if (propagation_steps[i] == BOUNDARY_STEPS_NONE) {
      factors[i] = 0.0f;
    }
  }
}

static void filter_verts_outside_symmetry_area(const Span<float3> positions,
                                               const float3 pivot,
                                               const ePaintSymmetryFlags symm,
                                               const MutableSpan<float> factors)
{
  BLI_assert(positions.size() == factors.size());
  for (const int i : factors.index_range()) {
    if (!SCULPT_check_vertex_pivot_symmetry(positions[i], pivot, symm)) {
      factors[i] = 0.0f;
    }
  }
}

static void calc_bend_transform(const Span<float3> positions,
                                const Span<float3> pivot_positions,
                                const Span<float3> pivot_axes,
                                const Span<float> factors,
                                const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == pivot_positions.size());
  BLI_assert(positions.size() == pivot_axes.size());
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = math::rotate_around_axis(
        positions[i], pivot_positions[i], pivot_axes[i], factors[i]);
  }
}

static void calc_slide_transform(const Span<float3> positions,
                                 const Span<float3> directions,
                                 const Span<float> factors,
                                 const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == directions.size());
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = positions[i] + (directions[i] * factors[i]);
  }
}

static void calc_inflate_transform(const Span<float3> positions,
                                   const Span<float3> normals,
                                   const Span<float> factors,
                                   const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == normals.size());
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = positions[i] + (normals[i] * factors[i]);
  }
}

static void calc_grab_transform(const Span<float3> positions,
                                const float3 grab_delta,
                                const Span<float> factors,
                                const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = positions[i] + (grab_delta * factors[i]);
  }
}

static void calc_twist_transform(const Span<float3> positions,
                                 const float3 pivot_point,
                                 const float3 pivot_axis,
                                 const Span<float> factors,
                                 const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = math::rotate_around_axis(positions[i], pivot_point, pivot_axis, factors[i]);
  }
}

static void calc_smooth_transform(const Span<float3> positions,
                                  const Span<float3> smooth_targets,
                                  const Span<float> factors,
                                  const MutableSpan<float3> transforms)
{
  BLI_assert(positions.size() == factors.size());
  BLI_assert(positions.size() == smooth_targets.size());
  BLI_assert(positions.size() == transforms.size());

  for (const int i : positions.index_range()) {
    transforms[i] = positions[i] + (smooth_targets[i] * factors[i]);
  }
}

};  // namespace boundary_cc

static void init_falloff(const Brush &brush, const float radius, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != BOUNDARY_STEPS_NONE) {
      boundary.edit_info.strength_factor[i] = BKE_brush_curve_strength(
          &brush, boundary.edit_info.propagation_steps_num[i], boundary.max_propagation_steps);
    }

    if (boundary.edit_info.original_vertex_i[i] == boundary.initial_vert_i) {
      /* All vertices that are propagated from the original vertex won't be affected by the
       * boundary falloff, so there is no need to calculate anything else. */
      continue;
    }

    const bool use_boundary_distances = brush.boundary_falloff_type !=
                                        BRUSH_BOUNDARY_FALLOFF_CONSTANT;

    if (!use_boundary_distances) {
      /* There are falloff modes that do not require to modify the previously calculated falloff
       * based on boundary distances. */
      continue;
    }

    const float boundary_distance = boundary.distance.lookup_default(
        boundary.edit_info.original_vertex_i[i], 0.0f);
    float falloff_distance = 0.0f;
    float direction = 1.0f;

    switch (brush.boundary_falloff_type) {
      case BRUSH_BOUNDARY_FALLOFF_RADIUS:
        falloff_distance = boundary_distance;
        break;
      case BRUSH_BOUNDARY_FALLOFF_LOOP: {
        const int div = boundary_distance / radius;
        const float mod = fmodf(boundary_distance, radius);
        falloff_distance = div % 2 == 0 ? mod : radius - mod;
        break;
      }
      case BRUSH_BOUNDARY_FALLOFF_LOOP_INVERT: {
        const int div = boundary_distance / radius;
        const float mod = fmodf(boundary_distance, radius);
        falloff_distance = div % 2 == 0 ? mod : radius - mod;
        /* Inverts the falloff in the intervals 1 2 5 6 9 10 ... etc. */
        if (((div - 1) & 2) == 0) {
          direction = -1.0f;
        }
        break;
      }
      case BRUSH_BOUNDARY_FALLOFF_CONSTANT:
        /* For constant falloff distances are not allocated, so this should never happen. */
        BLI_assert(false);
    }

    boundary.edit_info.strength_factor[i] *= direction * BKE_brush_curve_strength(
                                                             &brush, falloff_distance, radius);
  }
}

static void bend_data_init_mesh(const Span<float3> vert_positions,
                                const Span<float3> vert_normals,
                                SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();

  boundary.bend.pivot_rotation_axis = Array<float3>(num_elements, float3(0));
  boundary.bend.pivot_positions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }

    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];

    const float3 normal = vert_normals[i];
    const float3 dir = vert_positions[orig_vert_i] - vert_positions[i];
    boundary.bend.pivot_rotation_axis[orig_vert_i] = math::normalize(math::cross(dir, normal));
    boundary.bend.pivot_positions[orig_vert_i] = vert_positions[i];
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];

    boundary.bend.pivot_positions[i] = boundary.bend.pivot_positions[orig_vert_i];
    boundary.bend.pivot_rotation_axis[i] = boundary.bend.pivot_rotation_axis[orig_vert_i];
  }
}

static void bend_data_init_grids(const SubdivCCG &subdiv_ccg, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();

  const CCGKey &key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  Span<CCGElem *> grids = subdiv_ccg.grids;

  boundary.bend.pivot_rotation_axis = Array<float3>(num_elements, float3(0));
  boundary.bend.pivot_positions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }

    const SubdivCCGCoord vert = SubdivCCGCoord::from_index(key, i);
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    const SubdivCCGCoord orig_vert = SubdivCCGCoord::from_index(key, orig_vert_i);

    const float3 normal = CCG_grid_elem_no(key, grids[vert.grid_index], vert.x, vert.y);
    const float3 dir = CCG_grid_elem_co(
                           key, grids[orig_vert.grid_index], orig_vert.x, orig_vert.y) -
                       CCG_grid_elem_co(key, grids[vert.grid_index], vert.x, vert.y);
    boundary.bend.pivot_rotation_axis[orig_vert_i] = math::normalize(math::cross(dir, normal));
    boundary.bend.pivot_positions[orig_vert_i] = CCG_grid_elem_co(
        key, grids[vert.grid_index], vert.x, vert.y);
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];

    boundary.bend.pivot_positions[i] = boundary.bend.pivot_positions[orig_vert_i];
    boundary.bend.pivot_rotation_axis[i] = boundary.bend.pivot_rotation_axis[orig_vert_i];
  }
}

static void bend_data_init_bmesh(BMesh *bm, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();

  boundary.bend.pivot_rotation_axis = Array<float3>(num_elements, float3(0));
  boundary.bend.pivot_positions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }

    BMVert *vert = BM_vert_at_index(bm, i);
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    BMVert *orig_vert = BM_vert_at_index(bm, orig_vert_i);

    const float3 normal = vert->no;
    const float3 dir = float3(orig_vert->co) - float3(vert->co);
    boundary.bend.pivot_rotation_axis[orig_vert_i] = math::normalize(math::cross(dir, normal));
    boundary.bend.pivot_positions[boundary.edit_info.original_vertex_i[i]] = vert->co;
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    boundary.bend.pivot_positions[i] = boundary.bend.pivot_positions[orig_vert_i];
    boundary.bend.pivot_rotation_axis[i] = boundary.bend.pivot_rotation_axis[orig_vert_i];
  }
}

static void slide_data_init_mesh(const Span<float3> vert_positions, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();
  boundary.slide.directions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    boundary.slide.directions[orig_vert_i] = math::normalize(vert_positions[orig_vert_i] -
                                                             vert_positions[i]);
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    boundary.slide.directions[i] =
        boundary.slide.directions[boundary.edit_info.original_vertex_i[i]];
  }
}

static void slide_data_init_grids(const SubdivCCG &subdiv_ccg, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();
  const CCGKey &key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  Span<CCGElem *> grids = subdiv_ccg.grids;

  boundary.slide.directions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }
    const SubdivCCGCoord vert = SubdivCCGCoord::from_index(key, i);
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    const SubdivCCGCoord orig_vert = SubdivCCGCoord::from_index(key, orig_vert_i);

    boundary.slide.directions[orig_vert_i] = math::normalize(
        CCG_grid_elem_co(key, grids[orig_vert.grid_index], orig_vert.x, orig_vert.y) -
        CCG_grid_elem_co(key, grids[vert.grid_index], vert.x, vert.y));
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    boundary.slide.directions[i] =
        boundary.slide.directions[boundary.edit_info.original_vertex_i[i]];
  }
}

static void slide_data_init_bmesh(BMesh *bm, SculptBoundary &boundary)
{
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.propagation_steps_num.size());
  BLI_assert(boundary.edit_info.original_vertex_i.size() ==
             boundary.edit_info.strength_factor.size());

  const int num_elements = boundary.edit_info.strength_factor.size();
  boundary.slide.directions = Array<float3>(num_elements, float3(0));

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] != boundary.max_propagation_steps) {
      continue;
    }
    BMVert *vert = BM_vert_at_index(bm, i);
    const int orig_vert_i = boundary.edit_info.original_vertex_i[i];
    BMVert *orig_vert = BM_vert_at_index(bm, orig_vert_i);
    boundary.slide.directions[orig_vert_i] = math::normalize(float3(orig_vert->co) -
                                                             float3(vert->co));
  }

  for (const int i : IndexRange(num_elements)) {
    if (boundary.edit_info.propagation_steps_num[i] == BOUNDARY_STEPS_NONE) {
      continue;
    }
    boundary.slide.directions[i] =
        boundary.slide.directions[boundary.edit_info.original_vertex_i[i]];
  }
}

static void populate_twist_data(const Span<float3> positions, SculptBoundary &boundary)
{
  boundary.twist.pivot_position = float3(0);
  for (const float3 &position : positions) {
    boundary.twist.pivot_position += position;
  }
  boundary.twist.pivot_position *= 1.0f / boundary.verts.size();
  boundary.twist.rotation_axis = math::normalize(boundary.pivot_position -
                                                 boundary.initial_vert_position);
}

static void twist_data_init_mesh(const Span<float3> vert_positions, SculptBoundary &boundary)
{
  Array<float3> positions(boundary.verts.size());
  array_utils::gather(vert_positions, boundary.verts.as_span(), positions.as_mutable_span());
  populate_twist_data(positions, boundary);
}

static void twist_data_init_grids(const SubdivCCG &subdiv_ccg, SculptBoundary &boundary)
{
  const CCGKey &key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  const Span<CCGElem *> grids = subdiv_ccg.grids;

  Array<float3> positions(boundary.verts.size());
  for (const int i : positions.index_range()) {
    const SubdivCCGCoord vert = SubdivCCGCoord::from_index(key, boundary.verts[i]);
    positions[i] = CCG_grid_elem_co(key, grids[vert.grid_index], vert.x, vert.y);
  }
  populate_twist_data(positions, boundary);
}

static void twist_data_init_bmesh(BMesh *bm, SculptBoundary &boundary)
{
  Array<float3> positions(boundary.verts.size());
  for (const int i : positions.index_range()) {
    BMVert *vert = BM_vert_at_index(bm, i);
    positions[i] = vert->co;
  }
  populate_twist_data(positions, boundary);
}

static void init_boundary_mesh(Object &object,
                               const Brush &brush,
                               const ePaintSymmetryFlags symm_area)
{
  const SculptSession &ss = *object.sculpt;
  const bke::pbvh::Tree &pbvh = *ss.pbvh;

  const Mesh &mesh = *static_cast<const Mesh *>(object.data);
  const bke::AttributeAccessor attributes = mesh.attributes();
  VArraySpan<bool> hide_vert = *attributes.lookup<bool>(".hide_vert", bke::AttrDomain::Point);

  const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);
  const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);

  /* TODO: Remove PBVHVertRef here once we decide how we are storing the active_vertex value. */
  PBVHVertRef initial_vert_ref = SCULPT_active_vertex_get(ss);
  if (initial_vert_ref.i == PBVH_REF_NONE) {
    return;
  }

  std::optional<int> initial_vert;
  if (ss.cache->mirror_symmetry_pass == 0) {
    initial_vert = initial_vert_ref.i;
  }
  else {
    float3 location;
    flip_v3_v3(location, positions_eval[initial_vert_ref.i], symm_area);
    initial_vert = nearest_vert_calc_mesh(
        *ss.pbvh, positions_eval, hide_vert, location, ss.cache->radius_squared, false);
  }

  if (!initial_vert) {
    return;
  }

  ss.cache->boundaries[symm_area] = boundary::data_init_mesh(
      object, &brush, *initial_vert, ss.cache->initial_radius);

  if (ss.cache->boundaries[symm_area]) {
    switch (brush.boundary_deform_type) {
      case BRUSH_BOUNDARY_DEFORM_BEND:
        bend_data_init_mesh(positions_eval, vert_normals, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_EXPAND:
        slide_data_init_mesh(positions_eval, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_TWIST:
        twist_data_init_mesh(positions_eval, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_INFLATE:
      case BRUSH_BOUNDARY_DEFORM_GRAB:
      case BRUSH_BOUNDARY_DEFORM_SMOOTH:
        /* Do nothing. These deform modes don't need any extra data to be precomputed. */
        break;
      default:
        BLI_assert_unreachable();
        break;
    }

    init_falloff(brush, ss.cache->initial_radius, *ss.cache->boundaries[symm_area]);
  }
}

static void init_boundary_grids(Object &object,
                                const Brush &brush,
                                const ePaintSymmetryFlags symm_area)
{
  const SculptSession &ss = *object.sculpt;
  const bke::pbvh::Tree &pbvh = *ss.pbvh;

  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey &key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  Span<CCGElem *> grids = subdiv_ccg.grids;

  /* TODO: Remove PBVHVertRef here once we decide how we are storing the active_vertex value. */
  PBVHVertRef initial_vert_ref = SCULPT_active_vertex_get(ss);
  if (initial_vert_ref.i == PBVH_REF_NONE) {
    return;
  }

  std::optional<SubdivCCGCoord> initial_vert;
  if (ss.cache->mirror_symmetry_pass == 0) {
    initial_vert = SubdivCCGCoord::from_index(key, initial_vert_ref.i);
  }
  else {
    const SubdivCCGCoord active_vert = SubdivCCGCoord::from_index(key, initial_vert_ref.i);
    float3 location;
    flip_v3_v3(location,
               CCG_grid_elem_co(key, grids[active_vert.grid_index], active_vert.x, active_vert.y),
               symm_area);
    initial_vert = nearest_vert_calc_grids(
        pbvh, subdiv_ccg, location, ss.cache->radius_squared, false);
  }

  if (!initial_vert) {
    return;
  }

  ss.cache->boundaries[symm_area] = boundary::data_init_grids(
      object, &brush, *initial_vert, ss.cache->initial_radius);

  if (ss.cache->boundaries[symm_area]) {
    switch (brush.boundary_deform_type) {
      case BRUSH_BOUNDARY_DEFORM_BEND:
        bend_data_init_grids(subdiv_ccg, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_EXPAND:
        slide_data_init_grids(subdiv_ccg, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_TWIST:
        twist_data_init_grids(subdiv_ccg, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_INFLATE:
      case BRUSH_BOUNDARY_DEFORM_GRAB:
      case BRUSH_BOUNDARY_DEFORM_SMOOTH:
        /* Do nothing. These deform modes don't need any extra data to be precomputed. */
        break;
      default:
        BLI_assert_unreachable();
        break;
    }

    init_falloff(brush, ss.cache->initial_radius, *ss.cache->boundaries[symm_area]);
  }
}

static void init_boundary_bmesh(Object &object,
                                const Brush &brush,
                                const ePaintSymmetryFlags symm_area)
{
  const SculptSession &ss = *object.sculpt;
  const bke::pbvh::Tree &pbvh = *ss.pbvh;

  BMesh *bm = ss.bm;

  /* TODO: Remove PBVHVertRef here once we decide how we are storing the active_vertex value. */
  PBVHVertRef initial_vert_ref = SCULPT_active_vertex_get(ss);
  if (initial_vert_ref.i == PBVH_REF_NONE) {
    return;
  }

  std::optional<BMVert *> initial_vert;
  if (ss.cache->mirror_symmetry_pass == 0) {
    initial_vert = reinterpret_cast<BMVert *>(initial_vert_ref.i);
  }
  else {
    BMVert *active_vert = reinterpret_cast<BMVert *>(initial_vert_ref.i);
    float3 location;
    flip_v3_v3(location, active_vert->co, symm_area);
    initial_vert = nearest_vert_calc_bmesh(pbvh, location, ss.cache->radius_squared, false);
  }

  if (!initial_vert) {
    return;
  }

  ss.cache->boundaries[symm_area] = boundary::data_init_bmesh(
      object, &brush, *initial_vert, ss.cache->initial_radius);

  if (ss.cache->boundaries[symm_area]) {
    switch (brush.boundary_deform_type) {
      case BRUSH_BOUNDARY_DEFORM_BEND:
        bend_data_init_bmesh(bm, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_EXPAND:
        slide_data_init_bmesh(bm, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_TWIST:
        twist_data_init_bmesh(bm, *ss.cache->boundaries[symm_area]);
        break;
      case BRUSH_BOUNDARY_DEFORM_INFLATE:
      case BRUSH_BOUNDARY_DEFORM_GRAB:
      case BRUSH_BOUNDARY_DEFORM_SMOOTH:
        /* Do nothing. These deform modes don't need any extra data to be precomputed. */
        break;
      default:
        BLI_assert_unreachable();
        break;
    }

    init_falloff(brush, ss.cache->initial_radius, *ss.cache->boundaries[symm_area]);
  }
}

void do_boundary_brush(const Sculpt &sd, Object &ob, Span<bke::pbvh::Node *> nodes)
{
  SculptSession &ss = *ob.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);

  const ePaintSymmetryFlags symm_area = ss.cache->mirror_symmetry_pass;
  if (SCULPT_stroke_is_first_brush_step_of_symmetry_pass(*ss.cache)) {
    switch (ss.pbvh->type()) {
      case bke::pbvh::Type::Mesh:
        init_boundary_mesh(ob, brush, symm_area);
        break;
      case bke::pbvh::Type::Grids:
        init_boundary_grids(ob, brush, symm_area);
        break;
      case bke::pbvh::Type::BMesh:
        init_boundary_bmesh(ob, brush, symm_area);
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
  }

  /* No active boundary under the cursor. */
  if (!ss.cache->boundaries[symm_area]) {
    return;
  }

  switch (brush.boundary_deform_type) {
    case BRUSH_BOUNDARY_DEFORM_BEND:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          boundary_brush_bend_task(ob, brush, nodes[i]);
        }
      });
      break;
    case BRUSH_BOUNDARY_DEFORM_EXPAND:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          brush_slide_task(ob, brush, nodes[i]);
        }
      });
      break;
    case BRUSH_BOUNDARY_DEFORM_INFLATE:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          brush_inflate_task(ob, brush, nodes[i]);
        }
      });
      break;
    case BRUSH_BOUNDARY_DEFORM_GRAB:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          brush_grab_task(ob, brush, nodes[i]);
        }
      });
      break;
    case BRUSH_BOUNDARY_DEFORM_TWIST:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          brush_twist_task(ob, brush, nodes[i]);
        }
      });
      break;
    case BRUSH_BOUNDARY_DEFORM_SMOOTH:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          brush_smooth_task(ob, brush, nodes[i]);
        }
      });
      break;
  }
}

};  // namespace blender::ed::sculpt_paint
