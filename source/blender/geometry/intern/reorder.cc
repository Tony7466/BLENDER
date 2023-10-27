/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.h"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"

namespace blender::geometry {

template<typename T, typename Func>
static void parallel_transform(MutableSpan<T> values, const int64_t grain_size, const Func &func)
{
  threading::parallel_for(values.index_range(), grain_size, [&](const IndexRange range) {
    MutableSpan<T> values_range = values.slice(range);
    std::transform(values_range.begin(), values_range.end(), values_range.begin(), func);
  });
}

static Array<int> invert_permutation(const Span<int> permutation)
{
  Array<int> data(permutation.size());
  for (const int i : permutation.index_range()) {
    data[permutation[i]] = i;
  }
  return data;
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

static Array<int> make_new_offset_indices(const OffsetIndices<int> old_offsets,
                                          const Span<int> old_by_new_map)
{
  Array<int> new_offsets(old_offsets.data().size());
  new_offsets[0] = 0;
  for (const int new_i : old_offsets.index_range()) {
    const int old_i = old_by_new_map[new_i];
    new_offsets[new_i + 1] = new_offsets[new_i] + old_offsets[old_i].size();
  }
  return new_offsets;
}

static void reorder_customdata_groups(CustomData &data,
                                      const OffsetIndices<int> old_offsets,
                                      const OffsetIndices<int> new_offsets,
                                      const Span<int> new_by_old_map)
{
  const int elements_num = new_offsets.total_size();
  const int groups_num = new_by_old_map.size();
  CustomData new_data;
  CustomData_copy_layout(&data, &new_data, CD_MASK_ALL, CD_CONSTRUCT, elements_num);
  for (const int old_i : IndexRange(groups_num)) {
    const int new_i = new_by_old_map[old_i];
    const IndexRange old_range = old_offsets[old_i];
    const IndexRange new_range = new_offsets[new_i];
    BLI_assert(old_range.size() == new_range.size());
    CustomData_copy_data(&data, &new_data, old_range.start(), new_range.start(), old_range.size());
  }
  CustomData_free(&data, elements_num);
  data = new_data;
}

void reorder_mesh_verts(const Span<int> indices, Mesh &mesh)
{
  reorder_customdata(mesh.vert_data, indices);

  const auto old_index_to_new = [&](const int old_i) { return indices[old_i]; };
  parallel_transform(mesh.edges_for_write().cast<int>(), 4098, old_index_to_new);
  parallel_transform(mesh.corner_verts_for_write(), 4098, old_index_to_new);

  BKE_mesh_tag_topology_changed(&mesh);
}

void reorder_mesh_edges(const Span<int> indices, Mesh &mesh)
{
  reorder_customdata(mesh.edge_data, indices);

  parallel_transform(
      mesh.corner_edges_for_write(), 4098, [&](const int old_i) { return indices[old_i]; });

  BKE_mesh_tag_topology_changed(&mesh);
}

/*
static Array<int> make_new_offset_indices_2(const OffsetIndices<int> old_offsets,
                                          const Span<int> new_by_old_map)
{
  Array<int> new_offsets(old_offsets.data().size());
  array_utils::copy(old_offsets.data().size(), new_offsets.as_mutable_span());
  offset_indices::offsets_to_indices(new_offsets.as_mutable_span());

  for (const int new_i : old_offsets.index_range()) {
    new_offsets[new_by_old_map[new_i]] = old_offsets[new_i].size();
  }
  offset_indices::accumulate_offsets(new_offsets.as_mutable_span());
  return new_offsets;
}
*/

void reorder_mesh_faces(const Span<int> indices, Mesh &mesh)
{
  const Array<int> old_by_new_map = invert_permutation(indices);

  reorder_customdata(mesh.face_data, indices);

  const OffsetIndices old_faces = mesh.faces();
  Array<int> new_face_offsets = make_new_offset_indices(old_faces, old_by_new_map);
  const OffsetIndices<int> new_faces = new_face_offsets.as_span();

  reorder_customdata_groups(mesh.loop_data, old_faces, new_faces, indices);

  array_utils::copy(new_face_offsets.as_span(), mesh.face_offsets_for_write());

  BKE_mesh_tag_topology_changed(&mesh);
}

void reorder_points(const Span<int> indices, PointCloud &pointcloud)
{
  reorder_customdata(pointcloud.pdata, indices);

  pointcloud.tag_positions_changed();
  pointcloud.tag_radii_changed();
}

void reorder_curves(const Span<int> indices, bke::CurvesGeometry &curves)
{
  const Array<int> old_by_new_map = invert_permutation(indices);

  reorder_customdata(curves.curve_data, indices);

  const OffsetIndices old_points_by_curve = curves.points_by_curve();
  Array<int> new_curve_offsets = make_new_offset_indices(old_points_by_curve, old_by_new_map);
  const OffsetIndices<int> new_points_by_curve = new_curve_offsets.as_span();

  reorder_customdata_groups(curves.point_data, old_points_by_curve, new_points_by_curve, indices);

  array_utils::copy(new_curve_offsets.as_span(), curves.offsets_for_write());

  curves.tag_topology_changed();
}

void reorder_instaces(const Span<int> indices, bke::Instances &instances)
{
  const int instances_num = instances.instances_num();

  reorder_customdata(instances.custom_data_attributes(), indices);

  const Span<int> old_reference_handles = instances.reference_handles();
  const Span<float4x4> old_transforms = instances.transforms();

  Array<int> new_reference_handles(instances_num);
  array_utils::gather(old_reference_handles, indices, new_reference_handles.as_mutable_span());
  array_utils::copy(new_reference_handles.as_span(), instances.reference_handles());

  Array<float4x4> new_transforms(instances_num);
  array_utils::gather(old_transforms, indices, new_transforms.as_mutable_span());
  array_utils::copy(new_transforms.as_span(), instances.transforms());
}

}  // namespace blender::geometry
