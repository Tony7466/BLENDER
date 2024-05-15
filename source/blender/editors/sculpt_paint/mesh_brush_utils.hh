/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "DNA_brush_enums.h"

struct Brush;
struct Mesh;
struct Object;
struct PBVHNode;
struct Sculpt;
struct SculptSession;

namespace blender::ed::sculpt_paint {

namespace auto_mask {
struct Cache;
};

/**
 * Note on the various positions arrays:
 * - positions_sculpt: The positions affected by brush strokes (maybe indirectly). Owned by the
 *   PBVH or mesh.
 * - positions_mesh: Positions owned by the original mesh. Not the same as `positions_sculpt` if
 *   there are deform modifiers.
 * - positions_eval: Positions after procedural deformation, used to build the PBVH. Translations
 *   are built for these values, then applied to `positions_sculpt`.
 *
 * Only two of these arrays are actually necessary. The third comes from the fact that the PBVH
 * currently stores its own copy of positions when there are deformations. If that was removed, the
 * situation would be clearer.
 *
 * \todo Get rid of one of the arrays mentioned above to avoid the situation with evaluated
 * positions, original positions, and then a third copy that's just there because of historical
 * reasons. This would involve removing access to positions and normals from the PBVH structure,
 * which should only be concerned with splitting geometry into spacially contiguous chunks.
 */

void calc_mesh_hide_and_mask(const Mesh &mesh,
                             Span<int> vert_indices,
                             MutableSpan<float> r_factors);

void calc_distance_falloff(SculptSession &ss,
                           Span<float3> vert_positions,
                           Span<int> vert_indices,
                           eBrushFalloffShape falloff_shape,
                           MutableSpan<float> r_distances,
                           MutableSpan<float> factors);

void calc_brush_strength_factors(const SculptSession &ss,
                                 const Brush &brush,
                                 Span<int> vert_indices,
                                 Span<float> distances,
                                 MutableSpan<float> factors);

void calc_brush_texture_factors(SculptSession &ss,
                                const Brush &brush,
                                Span<float3> vert_positions,
                                Span<int> vert_indices,
                                MutableSpan<float> factors);

void calc_brush_texture_colors(SculptSession &ss,
                               const Brush &brush,
                               Span<float3> vert_positions,
                               Span<int> vert_indices,
                               Span<float> factors,
                               MutableSpan<float4> r_colors);

void calc_front_face(const float3 &view_normal,
                     Span<float3> vert_normals,
                     Span<int> vert_indices,
                     MutableSpan<float> factors);

/**
 * \todo Remove call to `undo::push_node` deep inside `calc_mesh_automask` so the object argument
 * can be const. That may (hopefully) require pulling out the undo node push into the code for each
 * brush. That should help clarify the code path for brushes, and various optimizations will depend
 * on brush implementations doing their own undo pushes.
 */
void calc_mesh_automask(Object &object,
                        const auto_mask::Cache &cache,
                        PBVHNode &node,
                        Span<int> verts,
                        MutableSpan<float> factors);

void apply_translations(Span<float3> translations, Span<int> verts, MutableSpan<float3> positions);

/**
 * \todo Don't invert `deform_imats` on object evaluation. Instead just invert them on-demand in
 * brush implementations. This would be better because only the inversions required for affected
 * vertices would be necessary.
 */
void apply_crazyspace_to_translations(Span<float3x3> deform_imats,
                                      Span<int> verts,
                                      MutableSpan<float3> translations);

void clip_and_lock_translations(const Sculpt &sd,
                                const SculptSession &ss,
                                Span<float3> positions,
                                Span<int> verts,
                                MutableSpan<float3> translations);

MutableSpan<float3> mesh_brush_positions_for_write(SculptSession &ss, Mesh &mesh);

void flush_positions_to_shape_keys(Object &object,
                                   Span<int> verts,
                                   Span<float3> positions,
                                   MutableSpan<float3> positions_mesh);

}  // namespace blender::ed::sculpt_paint
