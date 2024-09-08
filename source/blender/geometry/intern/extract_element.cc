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

}  // namespace blender::geometry
