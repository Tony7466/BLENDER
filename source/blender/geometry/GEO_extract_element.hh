/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_attribute_filter.hh"

#include "BLI_array.hh"
#include "BLI_index_mask_fwd.hh"

struct Mesh;
struct PointCloud;
struct Curves;

namespace blender::geometry {

Array<Mesh *> extract_vertex_meshes(const Mesh &mesh,
                                    const IndexMask &mask,
                                    const bke::AttributeFilter &attribute_filter);

Array<Mesh *> extract_edge_meshes(const Mesh &mesh,
                                  const IndexMask &mask,
                                  const bke::AttributeFilter &attribute_filter);

Array<Mesh *> extract_face_meshes(const Mesh &mesh,
                                  const IndexMask &mask,
                                  const bke::AttributeFilter &attribute_filter);

Array<PointCloud *> extract_single_points(const PointCloud &pointcloud,
                                          const IndexMask &mask,
                                          const bke::AttributeFilter &attribute_filter);

Array<Curves *> extract_single_point_curves(const Curves &curves,
                                            const IndexMask &mask,
                                            const bke::AttributeFilter &attribute_filter);

Array<Curves *> extract_single_curves(const Curves &curves,
                                      const IndexMask &mask,
                                      const bke::AttributeFilter &attribute_filter);

}  // namespace blender::geometry
