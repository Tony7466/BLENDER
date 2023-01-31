/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 *
 * Interface for loading of asset library data, such as the contained asset representations,
 * previews and catalogs.
 */

#pragma once

namespace blender::asset_system {

class AssetLibrary;

class AbstractAssetLibraryLoader {
 public:
  AbstractAssetLibraryLoader() = default;
  virtual ~AbstractAssetLibraryLoader() = default;
};

}  // namespace blender::asset_system
