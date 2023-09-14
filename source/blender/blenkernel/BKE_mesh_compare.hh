/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_mesh_types.hh"

/** \file
 * \ingroup bke
 */

namespace blender::bke::mesh {

enum class MeshMismatch : int8_t {
  NumVerts = 0,         /* The number of vertices is different. */
  NumEdges = 1,         /* The number of edges is different. */
  NumCorners = 2,       /* The number of corners is different. */
  NumFaces = 3,         /* The number of faces is different. */
  VertexAttributes = 4, /* The values of the vertex attributes are different. */
  EdgeAttributes = 5,   /* The values of the edge attributes are different. */
  CornerAttributes = 6, /* The values of the corner attributes are different. */
  FaceAttributes = 7,   /* The values of the face attributes are different. */
  EdgeTopology = 8,     /* The edge topology is different. */
  FaceTopology = 9,     /* The face topology is different. */
};

/**
 * Checks if the two meshes are the same, up to a change of indices. Two meshes are considered the
 * same, if, for each domain, there is a bijection between the two meshes such that the bijections
 * preserve connectivity.
 *
 * In general, determining if two graphs are isomorphic is a very difficult problem. Because we
 * have more information than just connectivity (attributes), we can compute it in a more
 * reasonable time in most cases.
 */
std::optional<MeshMismatch> meshes_isomorphic(const Mesh &mesh1, const Mesh &mesh2);

}  // namespace blender::bke::mesh
