/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_bvhutils.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_sample.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_rand.hh"
#include "BLI_task.hh"

namespace blender::bke::mesh_surface_sample {

template<typename T>
BLI_NOINLINE static void sample_point_attribute(const Span<int> corner_verts,
                                                const Span<MLoopTri> looptris,
                                                const Span<int> looptri_indices,
                                                const Span<float3> bary_coords,
                                                const VArray<T> &src,
                                                const IndexMask mask,
                                                const MutableSpan<T> dst)
{
  for (const int i : mask) {
    const MLoopTri &tri = looptris[looptri_indices[i]];
    dst[i] = attribute_math::mix3(bary_coords[i],
                                  src[corner_verts[tri.tri[0]]],
                                  src[corner_verts[tri.tri[1]]],
                                  src[corner_verts[tri.tri[2]]]);
  }
}

void sample_point_attribute(const Span<int> corner_verts,
                            const Span<MLoopTri> looptris,
                            const Span<int> looptri_indices,
                            const Span<float3> bary_coords,
                            const GVArray &src,
                            const IndexMask mask,
                            const GMutableSpan dst)
{
  BLI_assert(src.type() == dst.type());

  const CPPType &type = src.type();
  attribute_math::convert_to_static_type(type, [&](auto dummy) {
    using T = decltype(dummy);
    sample_point_attribute<T>(corner_verts,
                              looptris,
                              looptri_indices,
                              bary_coords,
                              src.typed<T>(),
                              mask,
                              dst.typed<T>());
  });
}

template<typename T>
BLI_NOINLINE static void sample_corner_attribute(const Span<MLoopTri> looptris,
                                                 const Span<int> looptri_indices,
                                                 const Span<float3> bary_coords,
                                                 const VArray<T> &src,
                                                 const IndexMask mask,
                                                 const MutableSpan<T> dst)
{
  for (const int i : mask) {
    const MLoopTri &tri = looptris[looptri_indices[i]];
    dst[i] = sample_corner_attribute_with_bary_coords(bary_coords[i], tri, src);
  }
}

void sample_corner_normals(const Span<MLoopTri> looptris,
                           const Span<int> looptri_indices,
                           const Span<float3> bary_coords,
                           const Span<float3> src,
                           const IndexMask mask,
                           const MutableSpan<float3> dst)
{
  for (const int i : mask) {
    const MLoopTri &tri = looptris[looptri_indices[i]];
    const float3 value = sample_corner_attribute_with_bary_coords(bary_coords[i], tri, src);
    dst[i] = math::normalize(value);
  }
}

void sample_corner_attribute(const Span<MLoopTri> looptris,
                             const Span<int> looptri_indices,
                             const Span<float3> bary_coords,
                             const GVArray &src,
                             const IndexMask mask,
                             const GMutableSpan dst)
{
  BLI_assert(src.type() == dst.type());

  const CPPType &type = src.type();
  attribute_math::convert_to_static_type(type, [&](auto dummy) {
    using T = decltype(dummy);
    sample_corner_attribute<T>(
        looptris, looptri_indices, bary_coords, src.typed<T>(), mask, dst.typed<T>());
  });
}

template<typename T>
void sample_face_attribute(const Span<MLoopTri> looptris,
                           const Span<int> looptri_indices,
                           const VArray<T> &src,
                           const IndexMask mask,
                           const MutableSpan<T> dst)
{
  for (const int i : mask) {
    const int looptri_index = looptri_indices[i];
    const MLoopTri &looptri = looptris[looptri_index];
    const int poly_index = looptri.poly;
    dst[i] = src[poly_index];
  }
}

void sample_face_attribute(const Span<MLoopTri> looptris,
                           const Span<int> looptri_indices,
                           const GVArray &src,
                           const IndexMask mask,
                           const GMutableSpan dst)
{
  BLI_assert(src.type() == dst.type());

  const CPPType &type = src.type();
  attribute_math::convert_to_static_type(type, [&](auto dummy) {
    using T = decltype(dummy);
    sample_face_attribute<T>(looptris, looptri_indices, src.typed<T>(), mask, dst.typed<T>());
  });
}

void sample_barycentric_weights(const Span<float3> vert_positions,
                                const Span<int> corner_verts,
                                const Span<MLoopTri> looptris,
                                const Span<int> looptri_indices,
                                const Span<float3> sample_positions,
                                const IndexMask mask,
                                MutableSpan<float3> bary_coords)
{
  for (const int i : mask) {
    const MLoopTri &tri = looptris[looptri_indices[i]];
    bary_coords[i] = compute_bary_coord_in_triangle(
        vert_positions, corner_verts, tri, sample_positions[i]);
  }
}

int sample_surface_points_spherical(RandomNumberGenerator &rng,
                                    const Mesh &mesh,
                                    const Span<int> looptri_indices_to_sample,
                                    const float3 &sample_pos,
                                    const float sample_radius,
                                    const float approximate_density,
                                    Vector<float3> &r_bary_coords,
                                    Vector<int> &r_looptri_indices,
                                    Vector<float3> &r_positions)
{
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<MLoopTri> looptris = mesh.looptris();

  const float sample_radius_sq = pow2f(sample_radius);
  const float sample_plane_area = M_PI * sample_radius_sq;
  /* Used for switching between two triangle sampling strategies. */
  const float area_threshold = sample_plane_area;

  const int old_num = r_bary_coords.size();

  for (const int looptri_index : looptri_indices_to_sample) {
    const MLoopTri &looptri = looptris[looptri_index];

    const float3 &v0 = positions[corner_verts[looptri.tri[0]]];
    const float3 &v1 = positions[corner_verts[looptri.tri[1]]];
    const float3 &v2 = positions[corner_verts[looptri.tri[2]]];

    const float looptri_area = area_tri_v3(v0, v1, v2);

    if (looptri_area < area_threshold) {
      /* The triangle is small compared to the sample radius. Sample by generating random
       * barycentric coordinates. */
      const int amount = rng.round_probabilistic(approximate_density * looptri_area);
      for ([[maybe_unused]] const int i : IndexRange(amount)) {
        const float3 bary_coord = rng.get_barycentric_coordinates();
        const float3 point_pos = attribute_math::mix3(bary_coord, v0, v1, v2);
        const float dist_to_sample_sq = math::distance_squared(point_pos, sample_pos);
        if (dist_to_sample_sq > sample_radius_sq) {
          continue;
        }

        r_bary_coords.append(bary_coord);
        r_looptri_indices.append(looptri_index);
        r_positions.append(point_pos);
      }
    }
    else {
      /* The triangle is large compared to the sample radius. Sample by generating random points
       * on the triangle plane within the sample radius. */
      float3 normal;
      normal_tri_v3(normal, v0, v1, v2);

      float3 sample_pos_proj = sample_pos;
      project_v3_plane(sample_pos_proj, normal, v0);

      const float proj_distance_sq = math::distance_squared(sample_pos_proj, sample_pos);
      const float sample_radius_factor_sq = 1.0f -
                                            std::min(1.0f, proj_distance_sq / sample_radius_sq);
      const float radius_proj_sq = sample_radius_sq * sample_radius_factor_sq;
      const float radius_proj = std::sqrt(radius_proj_sq);
      const float circle_area = M_PI * radius_proj_sq;

      const int amount = rng.round_probabilistic(approximate_density * circle_area);

      const float3 axis_1 = math::normalize(v1 - v0) * radius_proj;
      const float3 axis_2 = math::normalize(math::cross(axis_1, math::cross(axis_1, v2 - v0))) *
                            radius_proj;

      for ([[maybe_unused]] const int i : IndexRange(amount)) {
        const float r = std::sqrt(rng.get_float());
        const float angle = rng.get_float() * 2.0f * M_PI;
        const float x = r * std::cos(angle);
        const float y = r * std::sin(angle);
        const float3 point_pos = sample_pos_proj + axis_1 * x + axis_2 * y;
        if (!isect_point_tri_prism_v3(point_pos, v0, v1, v2)) {
          /* Sampled point is not in the triangle. */
          continue;
        }

        float3 bary_coord;
        interp_weights_tri_v3(bary_coord, v0, v1, v2, point_pos);

        r_bary_coords.append(bary_coord);
        r_looptri_indices.append(looptri_index);
        r_positions.append(point_pos);
      }
    }
  }
  return r_bary_coords.size() - old_num;
}

int sample_surface_points_projected(
    RandomNumberGenerator &rng,
    const Mesh &mesh,
    BVHTreeFromMesh &mesh_bvhtree,
    const float2 &sample_pos_re,
    const float sample_radius_re,
    const FunctionRef<void(const float2 &pos_re, float3 &r_start, float3 &r_end)>
        region_position_to_ray,
    const bool front_face_only,
    const int tries_num,
    const int max_points,
    Vector<float3> &r_bary_coords,
    Vector<int> &r_looptri_indices,
    Vector<float3> &r_positions)
{
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<MLoopTri> looptris = mesh.looptris();

  int point_count = 0;
  for ([[maybe_unused]] const int _ : IndexRange(tries_num)) {
    if (point_count == max_points) {
      break;
    }

    const float r = sample_radius_re * std::sqrt(rng.get_float());
    const float angle = rng.get_float() * 2.0f * M_PI;
    float3 ray_start, ray_end;
    const float2 pos_re = sample_pos_re + r * float2(std::cos(angle), std::sin(angle));
    region_position_to_ray(pos_re, ray_start, ray_end);
    const float3 ray_direction = math::normalize(ray_end - ray_start);

    BVHTreeRayHit ray_hit;
    ray_hit.dist = FLT_MAX;
    ray_hit.index = -1;
    BLI_bvhtree_ray_cast(mesh_bvhtree.tree,
                         ray_start,
                         ray_direction,
                         0.0f,
                         &ray_hit,
                         mesh_bvhtree.raycast_callback,
                         &mesh_bvhtree);

    if (ray_hit.index == -1) {
      continue;
    }

    if (front_face_only) {
      const float3 normal = ray_hit.no;
      if (math::dot(ray_direction, normal) >= 0.0f) {
        continue;
      }
    }

    const int looptri_index = ray_hit.index;
    const float3 pos = ray_hit.co;

    const float3 bary_coords = compute_bary_coord_in_triangle(
        positions, corner_verts, looptris[looptri_index], pos);

    r_positions.append(pos);
    r_bary_coords.append(bary_coords);
    r_looptri_indices.append(looptri_index);
    point_count++;
  }
  return point_count;
}

float3 compute_bary_coord_in_triangle(const Span<float3> vert_positions,
                                      const Span<int> corner_verts,
                                      const MLoopTri &looptri,
                                      const float3 &position)
{
  float3 bary_coords;
  interp_weights_tri_v3(bary_coords,
                        vert_positions[corner_verts[looptri.tri[0]]],
                        vert_positions[corner_verts[looptri.tri[1]]],
                        vert_positions[corner_verts[looptri.tri[2]]],
                        position);
  return bary_coords;
}

}  // namespace blender::bke::mesh_surface_sample
