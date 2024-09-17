/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_extract_elements.hh"

#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

namespace blender::geometry {

Array<Mesh *> extract_mesh_vertices(const Mesh &mesh,
                                    const IndexMask &mask,
                                    const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.verts_num);
  Array<Mesh *> meshes(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int vert_i, const int mesh_i) {
    Mesh *vert_mesh = BKE_mesh_new_nomain(1, 0, 0, 0);
    /* TODO: Propagate attributes from other domains. Same for other functions. */
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{vert_i},
                           vert_mesh->attributes_for_write());
    meshes[mesh_i] = vert_mesh;
  });

  return meshes;
}

Array<Mesh *> extract_mesh_edges(const Mesh &mesh,
                                 const IndexMask &mask,
                                 const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.edges_num);
  Array<Mesh *> meshes(mask.size(), nullptr);

  const Span<int2> src_edges = mesh.edges();
  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int edge_i, const int mesh_i) {
    Mesh *edge_mesh = BKE_mesh_new_nomain(2, 1, 0, 0);

    MutableSpan<int2> new_edges = edge_mesh->edges_for_write();
    new_edges[0] = {0, 1};

    const int2 &src_edge = src_edges[edge_i];
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>({src_edge[0], src_edge[1]}),
                           edge_mesh->attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Edge,
                           bke::AttrDomain::Edge,
                           bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                           Span<int>{edge_i},
                           edge_mesh->attributes_for_write());

    meshes[mesh_i] = edge_mesh;
  });

  return meshes;
}

Array<Mesh *> extract_mesh_faces(const Mesh &mesh,
                                 const IndexMask &mask,
                                 const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.faces_num);
  Array<Mesh *> meshes(mask.size(), nullptr);

  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();
  const OffsetIndices<int> src_faces = mesh.faces();

  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int face_i, const int mesh_i) {
    const IndexRange src_face = src_faces[face_i];
    const int verts_num = src_face.size();

    Mesh *face_mesh = BKE_mesh_new_nomain(verts_num, verts_num, 1, verts_num);

    MutableSpan<int2> new_edges = face_mesh->edges_for_write();
    MutableSpan<int> new_corner_verts = face_mesh->corner_verts_for_write();
    MutableSpan<int> new_corner_edges = face_mesh->corner_edges_for_write();
    MutableSpan<int> new_face_offsets = face_mesh->face_offsets_for_write();

    for (const int i : IndexRange(verts_num)) {
      new_edges[i] = {i, i + 1};
      new_corner_verts[i] = i;
      new_corner_edges[i] = i;
    }
    new_edges.last()[1] = 0;
    new_face_offsets[0] = 0;
    new_face_offsets[1] = verts_num;

    Array<int> old_corner_indices(verts_num);
    Array<int> old_edge_indices(verts_num);
    Array<int> old_vert_indices(verts_num);
    for (const int i : IndexRange(verts_num)) {
      const int src_corner_i = src_face[i];
      const int src_edge_i = src_corner_edges[src_corner_i];
      const int src_vert_i = src_corner_verts[src_corner_i];
      old_corner_indices[i] = src_corner_i;
      old_edge_indices[i] = src_edge_i;
      old_vert_indices[i] = src_vert_i;
    }

    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           old_vert_indices,
                           face_mesh->attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Edge,
                           bke::AttrDomain::Edge,
                           bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                           old_edge_indices,
                           face_mesh->attributes_for_write());
    bke::gather_attributes(
        src_attributes,
        bke::AttrDomain::Corner,
        bke::AttrDomain::Corner,
        bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_edge", ".corner_vert"}),
        old_corner_indices,
        face_mesh->attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Face,
                           bke::AttrDomain::Face,
                           attribute_filter,
                           Span<int>{face_i},
                           face_mesh->attributes_for_write());

    meshes[mesh_i] = face_mesh;
  });

  return meshes;
}

Array<PointCloud *> extract_pointcloud_points(const PointCloud &pointcloud,
                                              const IndexMask &mask,
                                              const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= pointcloud.totpoint);
  Array<PointCloud *> pointclouds(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = pointcloud.attributes();

  mask.foreach_index(GrainSize(32), [&](const int point_i, const int pointcloud_i) {
    PointCloud *new_pointcloud = BKE_pointcloud_new_nomain(1);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{point_i},
                           new_pointcloud->attributes_for_write());
    pointclouds[pointcloud_i] = new_pointcloud;
  });

  return pointclouds;
}

Array<Curves *> extract_curves_points(const Curves &curves,
                                      const IndexMask &mask,
                                      const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= curves.geometry.point_num);
  Array<Curves *> new_curves(mask.size(), nullptr);

  const bke::CurvesGeometry &src_curves = curves.geometry.wrap();
  const bke::AttributeAccessor src_attributes = src_curves.attributes();

  mask.foreach_index(GrainSize(32), [&](const int point_i, const int single_curve_i) {
    /* TODO: Use src curve type. */
    Curves *single_curve = bke::curves_new_nomain_single(1, CURVE_TYPE_POLY);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{point_i},
                           single_curve->geometry.wrap().attributes_for_write());
    new_curves[single_curve_i] = single_curve;
  });

  return new_curves;
}

Array<Curves *> extract_curves(const Curves &curves,
                               const IndexMask &mask,
                               const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= curves.geometry.curve_num);
  Array<Curves *> new_curves(mask.size(), nullptr);

  const bke::CurvesGeometry &src_curves = curves.geometry.wrap();
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  const OffsetIndices<int> src_points_by_curve = src_curves.points_by_curve();

  mask.foreach_index(GrainSize(32), [&](const int curve_i, const int single_curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const int points_num = src_points.size();
    Curves *single_curve = bke::curves_new_nomain_single(points_num, CURVE_TYPE_POLY);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           src_points,
                           single_curve->geometry.wrap().attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Curve,
                           bke::AttrDomain::Curve,
                           attribute_filter,
                           Span<int>{curve_i},
                           single_curve->geometry.wrap().attributes_for_write());
    single_curve->geometry.wrap().update_curve_types();
    new_curves[single_curve_i] = single_curve;
  });

  return new_curves;
}

Array<bke::Instances *> extract_instances(const bke::Instances &instances,
                                          const IndexMask &mask,
                                          const bke::AttributeFilter &attribute_filter)
{
  using bke::Instances;
  BLI_assert(mask.min_array_size() <= instances.instances_num());
  Array<Instances *> new_instances(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = instances.attributes();
  const Span<bke::InstanceReference> src_references = instances.references();
  const Span<int> src_reference_handles = instances.reference_handles();
  const Span<float4x4> src_transforms = instances.transforms();

  mask.foreach_index(GrainSize(32), [&](const int old_instance_i, const int new_instance_i) {
    const int old_handle = src_reference_handles[old_instance_i];
    const bke::InstanceReference &old_reference = src_references[old_handle];
    const float4x4 &old_transform = src_transforms[old_instance_i];

    Instances *single_instance = new Instances();
    const int new_handle = single_instance->add_new_reference(old_reference);
    single_instance->add_instance(new_handle, old_transform);

    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Instance,
                           bke::AttrDomain::Instance,
                           bke::attribute_filter_with_skip_ref(
                               attribute_filter, {".reference_index", "instance_transform"}),
                           Span<int>{old_instance_i},
                           single_instance->attributes_for_write());

    new_instances[new_instance_i] = single_instance;
  });

  return new_instances;
}

}  // namespace blender::geometry
