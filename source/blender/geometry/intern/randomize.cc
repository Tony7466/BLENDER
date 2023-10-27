/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <random>

#include "GEO_randomize.hh"
#include "GEO_reorder.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_global.h"
#include "BKE_instances.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"

namespace blender::geometry {

static Array<int> get_permutation(const int length, const int seed)
{
  Array<int> data(length);
  array_utils::fill_index_range<int>(data);
  std::shuffle(data.begin(), data.end(), std::default_random_engine(seed));
  return data;
}

/**
 * We can't use a fully random seed, because then the randomization wouldn't be deterministic,
 * which is important to avoid causing issues when determinism is expected. Using a single constant
 * seed is not ideal either, because then two geometries might be randomized equally or very
 * similar. Ideally, the seed would be a hash of everything that feeds into the geometry processing
 * algorithm before the randomization, but that's too expensive. Just use something simple but
 * correct for now.
 */
static int seed_from_mesh(const Mesh &mesh)
{
  return mesh.totvert;
}

static int seed_from_pointcloud(const PointCloud &pointcloud)
{
  return pointcloud.totpoint;
}

static int seed_from_curves(const bke::CurvesGeometry &curves)
{
  return curves.point_num;
}

static int seed_from_instances(const bke::Instances &instances)
{
  return instances.instances_num();
}

void debug_randomize_vert_order(Mesh *mesh)
{
  if (mesh == nullptr || !use_debug_randomization()) {
    return;
  }

  const int seed = seed_from_mesh(*mesh);
  const Array<int> new_by_old_map = get_permutation(mesh->totvert, seed);
  reorder_mesh_verts(new_by_old_map, *mesh);
}

void debug_randomize_edge_order(Mesh *mesh)
{
  if (mesh == nullptr || !use_debug_randomization()) {
    return;
  }

  const int seed = seed_from_mesh(*mesh);
  const Array<int> new_by_old_map = get_permutation(mesh->totedge, seed);
  reorder_mesh_edges(new_by_old_map, *mesh);
}

void debug_randomize_face_order(Mesh *mesh)
{
  if (mesh == nullptr || !use_debug_randomization()) {
    return;
  }

  const int seed = seed_from_mesh(*mesh);
  const Array<int> new_by_old_map = get_permutation(mesh->faces_num, seed);
  reorder_mesh_faces(new_by_old_map, *mesh);
}

void debug_randomize_point_order(PointCloud *pointcloud)
{
  if (pointcloud == nullptr || !use_debug_randomization()) {
    return;
  }

  const int seed = seed_from_pointcloud(*pointcloud);
  const Array<int> new_by_old_map = get_permutation(pointcloud->totpoint, seed);
  reorder_points(new_by_old_map, *pointcloud);
}

void debug_randomize_curve_order(bke::CurvesGeometry *curves)
{
  if (curves == nullptr || !use_debug_randomization()) {
    return;
  }

  const int seed = seed_from_curves(*curves);
  const Array<int> new_by_old_map = get_permutation(curves->curve_num, seed);
  reorder_curves(new_by_old_map, *curves);
}

void debug_randomize_mesh_order(Mesh *mesh)
{
  if (mesh == nullptr || !use_debug_randomization()) {
    return;
  }

  debug_randomize_vert_order(mesh);
  debug_randomize_edge_order(mesh);
  debug_randomize_face_order(mesh);
}

void debug_randomize_instance_order(bke::Instances *instances)
{
  if (instances == nullptr || !use_debug_randomization()) {
    return;
  }

  const int instances_num = instances->instances_num();
  const int seed = seed_from_instances(*instances);
  const Array<int> new_by_old_map = get_permutation(instances_num, seed);
  reorder_instaces(new_by_old_map, *instances);
}

bool use_debug_randomization()
{
  return G.randomize_geometry_element_order;
}

}  // namespace blender::geometry
