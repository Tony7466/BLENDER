/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BKE_volume_grid.hh"

#include "BLI_hash.hh"

namespace blender::bke::volume_file_cache {

GVolumeGrid get_grid_from_file(StringRef file_path, StringRef grid_name, int simplify_level = 0);

struct GridsFromFile {
  std::string error_message;
  openvdb::MetaMap::Ptr meta_data;
  Vector<GVolumeGrid> grids;
};

GridsFromFile get_all_grids_from_file(StringRef file_path, int simplify_level = 0);

void unload_unused();

}  // namespace blender::bke::volume_file_cache
