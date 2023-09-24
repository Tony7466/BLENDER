/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_mesh_types.hh"

/** \file
 * \ingroup bke
 */

namespace blender::bke::mesh {

enum class MeshMismatch : int8_t;

/**
 * Convert the mismatch to a human-readable string for display.
 */
const char *mismatch_to_string(const MeshMismatch &mismatch);

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
