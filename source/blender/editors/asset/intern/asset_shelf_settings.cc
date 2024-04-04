/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edasset
 *
 * Internal and external APIs for #AssetShelfSettings.
 */

#include <type_traits>

#include "AS_asset_catalog_path.hh"

#include "DNA_screen_types.h"

#include "BLO_read_write.hh"

#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_string_ref.hh"

#include "BKE_asset.hh"
#include "BKE_preferences.h"

#include "asset_shelf.hh"

using namespace blender;
using namespace blender::ed::asset;

AssetShelfSettings::AssetShelfSettings()
{
  memset(this, 0, sizeof(*this));
}

AssetShelfSettings::AssetShelfSettings(const AssetShelfSettings &other)
{
  operator=(other);
}

static void settings_free_enabled_catalogs(AssetShelfSettings &settings)
{
  BKE_asset_catalog_path_list_free(settings.enabled_catalog_paths);
  BLI_assert(BLI_listbase_is_empty(&settings.enabled_catalog_paths));
}

AssetShelfSettings &AssetShelfSettings::operator=(const AssetShelfSettings &other)
{
  /* Start with a shallow copy. */
  memcpy(this, &other, sizeof(AssetShelfSettings));

  next = prev = nullptr;

  if (active_catalog_path) {
    active_catalog_path = BLI_strdup(other.active_catalog_path);
  }
  settings_free_enabled_catalogs(*this);
  enabled_catalog_paths = BKE_asset_catalog_path_list_duplicate(other.enabled_catalog_paths);

  return *this;
}

AssetShelfSettings::~AssetShelfSettings()
{
  settings_free_enabled_catalogs(*this);
  MEM_delete(active_catalog_path);
}

namespace blender::ed::asset::shelf {

void settings_blend_write(BlendWriter *writer, const AssetShelfSettings &settings)
{
  BLO_write_struct(writer, AssetShelfSettings, &settings);

  BKE_asset_catalog_path_list_blend_write(writer, settings.enabled_catalog_paths);
  BLO_write_string(writer, settings.active_catalog_path);
}

void settings_blend_read_data(BlendDataReader *reader, AssetShelfSettings &settings)
{
  BKE_asset_catalog_path_list_blend_read_data(reader, settings.enabled_catalog_paths);
  BLO_read_data_address(reader, &settings.active_catalog_path);
}

void settings_set_active_catalog(AssetShelfSettings &settings,
                                 const asset_system::AssetCatalogPath &path)
{
  MEM_delete(settings.active_catalog_path);
  settings.active_catalog_path = BLI_strdupn(path.c_str(), path.length());
}

void settings_set_all_catalog_active(AssetShelfSettings &settings)
{
  MEM_delete(settings.active_catalog_path);
  settings.active_catalog_path = nullptr;
}

bool settings_is_active_catalog(const AssetShelfSettings &settings,
                                const asset_system::AssetCatalogPath &path)
{
  return settings.active_catalog_path && settings.active_catalog_path == path.str();
}

bool settings_is_all_catalog_active(const AssetShelfSettings &settings)
{
  return !settings.active_catalog_path || !settings.active_catalog_path[0];
}

void settings_clear_enabled_catalogs(AssetShelf &shelf)
{
  settings_free_enabled_catalogs(shelf.settings);
  BKE_preferences_asset_shelf_settings_clear_enabled_catalog_paths(&U, shelf.idname);
}

bool settings_is_catalog_path_enabled(const AssetShelf &shelf,
                                      const asset_system::AssetCatalogPath &path)
{
  return BKE_preferences_asset_shelf_settings_is_catalog_path_enabled(
      &U, shelf.idname, path.c_str());
}

void settings_set_catalog_path_enabled(const AssetShelf &shelf,
                                       const asset_system::AssetCatalogPath &path)
{
  if (BKE_preferences_asset_shelf_settings_ensure_catalog_path_enabled(
          &U, shelf.idname, path.c_str()))
  {
    U.runtime.is_dirty = true;
  }
}

void settings_foreach_enabled_catalog_path(
    const AssetShelf &shelf,
    FunctionRef<void(const asset_system::AssetCatalogPath &catalog_path)> fn)
{
  const bUserAssetShelfSettings *pref_settings = BKE_preferences_asset_shelf_settings_get(
      &U, shelf.idname);
  if (!pref_settings) {
    return;
  }

  LISTBASE_FOREACH (AssetCatalogPathLink *, path_link, &pref_settings->enabled_catalog_paths) {
    fn(asset_system::AssetCatalogPath(path_link->path));
  }
}

}  // namespace blender::ed::asset::shelf
