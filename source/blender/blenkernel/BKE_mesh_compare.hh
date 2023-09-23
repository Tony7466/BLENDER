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
  VertexAttributes = 4, /* Some values of the vertex attributes are different. */
  EdgeAttributes = 5,   /* Some values of the edge attributes are different. */
  CornerAttributes = 6, /* Some values of the corner attributes are different. */
  FaceAttributes = 7,   /* Some values of the face attributes are different. */
  EdgeTopology = 8,     /* The edge topology is different. */
  FaceTopology = 9,     /* The face topology is different. */
  Attributes = 10,      /* The sets of attribute ids are different. */
  AttributeTypes = 11,  /* Some attributes with the same name have different types. */
};

/**
 * \brief Checks if the two meshes are the same, up to a change of indices.
 *
 * \details Two meshes are considered the same, if, for each domain, there is a bijection between
 * the two meshes such that the bijections preserve connectivity.
 *
 * In general, determining if two graphs are isomorphic is a very difficult problem (no polynomial
 * time algorithm is known). Because we have more information than just connectivity (attributes),
 * we can compute it in a more reasonable time in most cases.
 *
 * \warning This assumes that the mesh is of decent quality: no zero-size edges or faces.
 */
std::optional<MeshMismatch> meshes_isomorphic(const Mesh &mesh1, const Mesh &mesh2);

}  // namespace blender::bke::mesh
