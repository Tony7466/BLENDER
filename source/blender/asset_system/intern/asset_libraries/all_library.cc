/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#include "asset_library_service.hh"

#include "all_library.hh"

namespace blender::asset_system {

AllAssetLibrary::AllAssetLibrary() : AssetLibrary(ASSET_LIBRARY_ALL) {}

void AllAssetLibrary::load(const Main &bmain)
{
  AssetLibraryService::get_asset_library_all(&bmain);
}

}  // namespace blender::asset_system
