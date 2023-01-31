/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 *
 * Highest level asset system functions to manage asset libraries.
 */

#pragma once

#include <memory>

#include "BLI_string_ref.hh"

struct bUserAssetLibrary;

namespace blender::asset_system {

class AbstractAssetLibraryLoader;

void register_asset_library(StringRef ui_name,
                            StringRef identifier,
                            std::unique_ptr<AbstractAssetLibraryLoader> loader);
/**
 * Register a #bUserAssetLibrary reference (describing a library on disk) in the asset system. The
 * reference is not owning, so it needs to be unregistered before freeing. Benefit is that the
 * contained name and path don't need to be kept in sync.
 */
void register_asset_library(bUserAssetLibrary &on_disk_library,
                            std::unique_ptr<AbstractAssetLibraryLoader> loader);
void unregister_asset_library(StringRef identifier);
void unregister_asset_library(bUserAssetLibrary &on_disk_library);

}  // namespace blender::asset_system
