/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <iostream>
#include <random>

#include "GEO_randomize.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_customdata.h"
#include "BKE_mesh.hh"

#include "BLI_array.hh"

namespace blender::geometry {

static Array<int> get_permutation(const int length, const int seed)
{
  Array<int> data(length);
  for (const int i : IndexRange(length)) {
    data[i] = i;
  }
  std::shuffle(data.begin(), data.end(), std::default_random_engine(seed));
  return data;
}

static int seed_from_mesh(const Mesh &mesh)
{
  return mesh.totvert;
}

void randomize_vertex_order(Mesh &mesh)
{
  const int seed = seed_from_mesh(mesh);
  const Array<int> new_by_old_map = get_permutation(mesh.totvert, seed);

  CustomData new_vertex_data;
  CustomData_copy_layout(
      &mesh.vert_data, &new_vertex_data, CD_MASK_ALL, CD_CONSTRUCT, mesh.totvert);

  for (const int old_i : IndexRange(mesh.totvert)) {
    const int new_i = new_by_old_map[old_i];
    CustomData_copy_data(&mesh.vert_data, &new_vertex_data, old_i, new_i, 1);
  }
  CustomData_free(&mesh.vert_data, mesh.totvert);
  mesh.vert_data = new_vertex_data;

  for (int &v : mesh.edges_for_write().cast<int>()) {
    v = new_by_old_map[v];
  }
  for (int &v : mesh.corner_verts_for_write()) {
    v = new_by_old_map[v];
  }
}

}  // namespace blender::geometry
