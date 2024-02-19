/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#pragma once

#include "AS_asset_library.hh"

namespace blender::asset_system {

class AllAssetLibrary : public AssetLibrary {
  bool catalogs_dirty_ = true;

 public:
  AllAssetLibrary();

  void refresh_catalogs() override;

  void rebuild(const bool reload_catalogs);

  void tag_catalogs_dirty();
  bool is_catalogs_dirty();
};

}  // namespace blender::asset_system
