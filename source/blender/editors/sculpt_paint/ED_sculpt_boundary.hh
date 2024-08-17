/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 */
#pragma once

#include <memory>

#include "BLI_bit_span.hh"
#include "BLI_span.hh"
#include "BLI_offset_indices.hh"

struct Brush;
struct BMVert;
struct Depsgraph;
struct Object;
struct PBVHVertRef;
struct Sculpt;
struct SculptBoundary;
struct SculptBoundaryPreview;
struct SculptSession;
struct SubdivCCG;
struct SubdivCCGCoord;
namespace blender::bke::pbvh {
class Node;
}

namespace blender::ed::sculpt_paint::boundary {

/**
 * Populates boundary information for a mesh.
 *
 * \see SculptVertexInfo
 */
void ensure_boundary_info(Object &object);

/**
 * Determine if a vertex is a boundary vertex.
 *
 * Requires #ensure_boundary_info to have been called.
 */
bool vert_is_boundary(const SculptSession &ss, PBVHVertRef vertex);
bool vert_is_boundary(Span<bool> hide_poly,
                      GroupedSpan<int> vert_to_face_map,
                      BitSpan boundary,
                      int vert);
bool vert_is_boundary(const SubdivCCG &subdiv_ccg,
                      Span<int> corner_verts,
                      OffsetIndices<int> faces,
                      BitSpan boundary,
                      SubdivCCGCoord vert);
bool vert_is_boundary(BMVert *vert);


/**
 * Main function to get #SculptBoundary data both for brush deformation and viewport preview.
 * Can return NULL if there is no boundary from the given vertex using the given radius.
 */
std::unique_ptr<SculptBoundary> data_init(const Depsgraph &depsgraph,
                                          Object &object,
                                          const Brush *brush,
                                          PBVHVertRef initial_vert,
                                          float radius);
std::unique_ptr<SculptBoundary> data_init_mesh(const Depsgraph &depsgraph,
                                               Object &object,
                                               const Brush *brush,
                                               int initial_vert,
                                               float radius);
std::unique_ptr<SculptBoundary> data_init_grids(Object &object,
                                                const Brush *brush,
                                                SubdivCCGCoord initial_vert,
                                                float radius);
std::unique_ptr<SculptBoundary> data_init_bmesh(Object &object,
                                                const Brush *brush,
                                                BMVert *initial_vert,
                                                float radius);
std::unique_ptr<SculptBoundaryPreview> preview_data_init(const Depsgraph &depsgraph,
                                                         Object &object,
                                                         const Brush *brush,
                                                         float radius);

/* Main Brush Function. */
void do_boundary_brush(const Depsgraph &depsgraph,
                       const Sculpt &sd,
                       Object &ob,
                       Span<bke::pbvh::Node *> nodes);

void edges_preview_draw(uint gpuattr,
                        SculptSession &ss,
                        const float outline_col[3],
                        float outline_alpha);
void pivot_line_preview_draw(uint gpuattr, SculptSession &ss);

}
