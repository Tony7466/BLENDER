/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_extract_element.hh"

#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

namespace blender::geometry {

Array<Mesh *> extract_vertex_meshes(const Mesh &mesh,
                                    const IndexMask &mask,
                                    const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.verts_num);
  Array<Mesh *> meshes(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int vert_i, const int mesh_i) {
    Mesh *vert_mesh = BKE_mesh_new_nomain(1, 0, 0, 0);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{vert_i},
                           vert_mesh->attributes_for_write());
    meshes[mesh_i] = vert_mesh;
  });

  return meshes;
}

Array<Mesh *> extract_edge_meshes(const Mesh &mesh,
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
                           attribute_filter,
                           Span<int>({src_edge[0], src_edge[1]}),
                           edge_mesh->attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Edge,
                           bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                           Span<int>{edge_i},
                           edge_mesh->attributes_for_write());

    meshes[mesh_i] = edge_mesh;
  });

  return meshes;
}

}  // namespace blender::geometry
