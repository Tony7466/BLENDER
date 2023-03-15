/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#pragma once

#include "BLI_fileops.hh"

#include "ply_data.hh"

namespace blender::io::ply {

/**
 * Loads the information from a PLY file to a #PlyData data-structure.
 * \param file: The PLY file that was opened.
 * \param header: The information in the PLY header.
 * \return The #PlyData data-structure that can be used for conversion to a Mesh.
 */
std::unique_ptr<PlyData> import_ply_data(fstream &file, const PlyHeader &header);

}  // namespace blender::io::ply
