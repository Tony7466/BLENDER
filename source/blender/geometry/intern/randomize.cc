/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <iostream>
#include <random>

#include "GEO_randomize.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

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

static int seed_from_pointcloud(const PointCloud &pointcloud)
{
  return pointcloud.totpoint;
}

static void reorder_customdata(CustomData &data, const Span<int> new_by_old_map)
{
  CustomData new_data;
  CustomData_copy_layout(&data, &new_data, CD_MASK_ALL, CD_CONSTRUCT, new_by_old_map.size());

  for (const int old_i : new_by_old_map.index_range()) {
    const int new_i = new_by_old_map[old_i];
    CustomData_copy_data(&data, &new_data, old_i, new_i, 1);
  }
  CustomData_free(&data, new_by_old_map.size());
  data = new_data;
}

void randomize_vertex_order(Mesh &mesh)
{
  const int seed = seed_from_mesh(mesh);
  const Array<int> new_by_old_map = get_permutation(mesh.totvert, seed);

  reorder_customdata(mesh.vert_data, new_by_old_map);

  for (int &v : mesh.edges_for_write().cast<int>()) {
    v = new_by_old_map[v];
  }
  for (int &v : mesh.corner_verts_for_write()) {
    v = new_by_old_map[v];
  }
}

void randomize_edge_order(Mesh &mesh)
{
  const int seed = seed_from_mesh(mesh);
  const Array<int> new_by_old_map = get_permutation(mesh.totedge, seed);

  reorder_customdata(mesh.edge_data, new_by_old_map);

  for (int &e : mesh.corner_edges_for_write()) {
    e = new_by_old_map[e];
  }
}

void randomize_face_order(Mesh &mesh)
{
  const int seed = seed_from_mesh(mesh);
  const Array<int> new_by_old_map = get_permutation(mesh.faces_num, seed);
  Array<int> old_by_new_map(mesh.faces_num);
  for (const int old_i : IndexRange(mesh.faces_num)) {
    const int new_i = new_by_old_map[old_i];
    old_by_new_map[new_i] = old_i;
  }

  reorder_customdata(mesh.face_data, new_by_old_map);

  const OffsetIndices old_faces = mesh.faces();
  Array<int> new_face_offsets(mesh.faces_num + 1);
  new_face_offsets[0] = 0;
  for (const int new_i : IndexRange(mesh.faces_num)) {
    const int old_i = old_by_new_map[new_i];
    new_face_offsets[new_i + 1] = new_face_offsets[new_i] + old_faces[old_i].size();
  }
  const OffsetIndices<int> new_faces = new_face_offsets.as_span();

  {
    CustomData new_loop_data;
    CustomData_copy_layout(
        &mesh.loop_data, &new_loop_data, CD_MASK_ALL, CD_CONSTRUCT, mesh.totloop);
    for (const int old_face_i : IndexRange(mesh.faces_num)) {
      const int new_face_i = new_by_old_map[old_face_i];
      const IndexRange old_range = old_faces[old_face_i];
      const IndexRange new_range = new_faces[new_face_i];
      BLI_assert(old_range.size() == new_range.size());
      CustomData_copy_data(
          &mesh.loop_data, &new_loop_data, old_range.start(), new_range.start(), old_range.size());
    }
    CustomData_free(&mesh.loop_data, mesh.totloop);
    mesh.loop_data = new_loop_data;
  }

  mesh.face_offsets_for_write().copy_from(new_face_offsets);
}

void randomize_point_order(PointCloud &pointcloud)
{
  const int seed = seed_from_pointcloud(pointcloud);
  const Array<int> new_by_old_map = get_permutation(pointcloud.totpoint, seed);

  reorder_customdata(pointcloud.pdata, new_by_old_map);
}

}  // namespace blender::geometry
