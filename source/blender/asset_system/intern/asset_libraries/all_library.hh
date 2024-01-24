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
 public:
  AllAssetLibrary();
  void load(const Main &bmain) override;
};

}  // namespace blender::asset_system
