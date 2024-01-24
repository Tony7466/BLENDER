
/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#include "asset_library_service.hh"

#include "current_file_library.hh"

namespace blender::asset_system {

CurrentFileAssetLibrary::CurrentFileAssetLibrary() : {}

void CurrentFileAssetLibrary::load(const Main &bmain)
{
  /* For the "Current File" library  we get the asset library root path based on main. */
  std::string root_path = bmain ? AS_asset_library_find_suitable_root_path_from_main(bmain) : "";

  if (root_path.empty()) {
    /* File wasn't saved yet. */
    return AssetLibraryService::get_asset_library_current_file();
  }
  return AssetLibraryService::get_asset_library_on_disk_builtin(ASSET_LIBRARY_LOCAL, root_path);
}

}  // namespace blender::asset_system
