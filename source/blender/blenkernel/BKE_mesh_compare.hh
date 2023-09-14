/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_mesh_types.hh"

/** \file
 * \ingroup bke
 */

namespace blender::bke::mesh {
/**
 * Checks if the two meshes are the same, up to a change of indices. Two meshes are considered the
 * same, if, for each domain, there is a bijection between the two meshes such that the bijections
 * preserve connectivity.
 *
 * In general, determining if two graphs are isomorphic is a very difficult problem. Because we
 * have more information than just connectivity (attributes), we can compute it in a more
 * reasonable time in most cases.
 */
bool meshes_isomorphic(const Mesh &mesh1, const Mesh &mesh2);

}  // namespace blender::bke::mesh
