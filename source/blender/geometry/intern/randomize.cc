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

void randomize_edge_order(Mesh &mesh)
{
  const int seed = seed_from_mesh(mesh);
  const Array<int> new_by_old_map = get_permutation(mesh.totedge, seed);

  CustomData new_edge_data;
  CustomData_copy_layout(&mesh.edge_data, &new_edge_data, CD_MASK_ALL, CD_CONSTRUCT, mesh.totedge);

  for (const int old_i : IndexRange(mesh.totedge)) {
    const int new_i = new_by_old_map[old_i];
    CustomData_copy_data(&mesh.edge_data, &new_edge_data, old_i, new_i, 1);
  }
  CustomData_free(&mesh.edge_data, mesh.totedge);
  mesh.edge_data = new_edge_data;

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

  {
    CustomData new_face_data;
    CustomData_copy_layout(
        &mesh.face_data, &new_face_data, CD_MASK_ALL, CD_CONSTRUCT, mesh.faces_num);
    for (const int old_i : IndexRange(mesh.faces_num)) {
      const int new_i = new_by_old_map[old_i];
      CustomData_copy_data(&mesh.face_data, &new_face_data, old_i, new_i, 1);
    }
    CustomData_free(&mesh.face_data, mesh.faces_num);
    mesh.face_data = new_face_data;
  }

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

}  // namespace blender::geometry
