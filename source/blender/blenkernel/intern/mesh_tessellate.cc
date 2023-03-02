/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2001-2002 NaN Holding BV. All rights reserved. */

/** \file
 * \ingroup bke
 *
 * This file contains code for polygon tessellation
 * (creating triangles from polygons).
 *
 * \see bmesh_mesh_tessellate.c for the #BMesh equivalent of this file.
 */

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math.h"
#include "BLI_polyfill_2d.h"
#include "BLI_task.hh"

#include "BKE_mesh.hh"

#include "BLI_strict_flags.h"

namespace blender::bke::mesh {

struct PolyFillData {
  Vector<uint3> tris;
  Vector<float2> proj_verts;
};

/* -------------------------------------------------------------------- */
/** \name Loop Tessellation
 *
 * Fill in #MLoopTri data-structure.
 * \{ */

static MLoopTri looptri_init(
    const IndexRange poly, const uint poly_index, const uint a, const uint b, const uint c)
{
  MLoopTri tri{};
  tri.tri[0] = uint(poly.start() + a);
  tri.tri[1] = uint(poly.start() + b);
  tri.tri[2] = uint(poly.start() + c);
  tri.poly = poly_index;
  return tri;
}

/**
 * \param face_normal: This will be optimized out as a constant.
 */
BLI_INLINE void mesh_calc_tessellation_for_face_impl(const uint poly_index,
                                                     const IndexRange poly,
                                                     const Span<float3> positions,
                                                     const Span<MLoop> loops,
                                                     const float3 *normal_precalc,
                                                     PolyFillData &poly_fill_data,
                                                     MLoopTri *mlt)
{
  switch (poly.size()) {
    case 3: {
      *mlt = looptri_init(poly, poly_index, 0, 1, 2);
      break;
    }
    case 4: {
      mlt[0] = looptri_init(poly, poly_index, 0, 1, 2);
      mlt[1] = looptri_init(poly, poly_index, 0, 2, 3);

      if (UNLIKELY(normal_precalc ? is_quad_flip_v3_first_third_fast_with_normal(
                                        /* Simpler calculation (using the normal). */
                                        positions[loops[mlt[0].tri[0]].v],
                                        positions[loops[mlt[0].tri[1]].v],
                                        positions[loops[mlt[0].tri[2]].v],
                                        positions[loops[mlt[1].tri[2]].v],
                                        *normal_precalc) :
                                    is_quad_flip_v3_first_third_fast(
                                        /* Expensive calculation (no normal). */
                                        positions[loops[mlt[0].tri[0]].v],
                                        positions[loops[mlt[0].tri[1]].v],
                                        positions[loops[mlt[0].tri[2]].v],
                                        positions[loops[mlt[1].tri[2]].v]))) {
        /* Flip out of degenerate 0-2 state. */
        mlt[0].tri[2] = mlt[1].tri[2];
        mlt[1].tri[0] = mlt[0].tri[1];
      }
      break;
    }
    default: {
      float axis_mat[3][3];

      /* Calculate `axis_mat` to project verts to 2D. */
      if (normal_precalc == nullptr) {
        float3 normal(0);

        /* Calc normal, flipped: to get a positive 2D cross product. */
        const float *co_prev = positions[loops[poly.last()].v];
        for (const int64_t i : IndexRange(poly.size())) {
          const float *co_curr = positions[loops[poly[i]].v];
          add_newell_cross_v3_v3v3(normal, co_prev, co_curr);
          co_prev = co_curr;
        }
        if (UNLIKELY(normalize_v3(normal) == 0.0f)) {
          normal[2] = 1.0f;
        }
        axis_dominant_v3_to_m3_negate(axis_mat, normal);
      }
      else {
        axis_dominant_v3_to_m3_negate(axis_mat, *normal_precalc);
      }

      poly_fill_data.tris.reinitialize(poly.size() - 2);
      poly_fill_data.proj_verts.reinitialize(poly.size());

      MutableSpan<uint3> tris = poly_fill_data.tris;
      MutableSpan<float2> proj_verts = poly_fill_data.proj_verts;
      for (const int64_t i : IndexRange(poly.size())) {
        mul_v2_m3v3(proj_verts[i], axis_mat, positions[loops[poly[i]].v]);
      }

      BLI_polyfill_calc(reinterpret_cast<const float(*)[2]>(proj_verts.data()),
                        uint(poly.size()),
                        1,
                        reinterpret_cast<unsigned int(*)[3]>(tris.data()));

      /* Apply fill. */
      for (const int64_t i : tris.index_range()) {
        mlt[i] = looptri_init(poly, poly_index, tris[i][0], tris[i][1], tris[i][2]);
      }
      break;
    }
  }
}

static void looptris_calc_all(const Span<float3> positions,
                              const Span<MPoly> polys,
                              const Span<MLoop> loops,
                              const Span<float3> poly_normals,
                              MutableSpan<MLoopTri> looptris)
{
  threading::EnumerableThreadSpecific<PolyFillData> poly_fill_data;
  if (poly_normals.is_empty()) {
    threading::parallel_for(polys.index_range(), 1024L, [&](const IndexRange range) {
      PolyFillData &fill_data = poly_fill_data.local();
      for (const int64_t i : range) {
        const IndexRange poly(polys[i].loopstart, polys[i].totloop);
        const int tri_index = poly_to_tri_count(int(i), int(poly.start()));
        mesh_calc_tessellation_for_face_impl(
            uint(i), poly, positions, loops, nullptr, fill_data, &looptris[tri_index]);
      }
    });
  }
  else {
    threading::parallel_for(polys.index_range(), 1024L, [&](const IndexRange range) {
      PolyFillData &fill_data = poly_fill_data.local();
      for (const int64_t i : range) {
        const IndexRange poly(polys[i].loopstart, polys[i].totloop);
        const int tri_index = poly_to_tri_count(int(i), int(poly.start()));
        mesh_calc_tessellation_for_face_impl(
            uint(i), poly, positions, loops, &poly_normals[i], fill_data, &looptris[tri_index]);
      }
    });
  }
}

void looptris_calc(const Span<float3> vert_positions,
                   const Span<MPoly> polys,
                   const Span<MLoop> loops,
                   MutableSpan<MLoopTri> looptris)
{
  looptris_calc_all(vert_positions, polys, loops, {}, looptris);
}

void looptris_calc_with_normals(const Span<float3> vert_positions,
                                const Span<MPoly> polys,
                                const Span<MLoop> loops,
                                const Span<float3> poly_normals,
                                MutableSpan<MLoopTri> looptris)
{
  BLI_assert(!poly_normals.is_empty());
  looptris_calc_all(vert_positions, polys, loops, poly_normals, looptris);
}

/** \} */

}  // namespace blender::bke::mesh

void BKE_mesh_recalc_looptri(const MLoop *mloop,
                             const MPoly *polys,
                             const float (*vert_positions)[3],
                             int totvert,
                             int totloop,
                             int totpoly,
                             MLoopTri *mlooptri)
{
  blender::bke::mesh::looptris_calc(
      {reinterpret_cast<const blender::float3 *>(vert_positions), totvert},
      {polys, totpoly},
      {mloop, totloop},
      {mlooptri, poly_to_tri_count(totpoly, totloop)});
}
