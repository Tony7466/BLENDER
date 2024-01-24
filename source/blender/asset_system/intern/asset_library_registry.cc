/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#include <optional>
#include <string>

#include "DNA_userdef_types.h"

#include "BLI_listbase.h"

#include "AS_asset_library.hh"
#include "AS_essentials_library.hh"
#include "asset_libraries/all_library.hh"
#include "asset_libraries/current_file_library.hh"

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "CLG_log.h"

static CLG_LogRef LOG = {"asset_system"};

namespace blender::asset_system {

class AssetLibraryRegistry {
 public:
  using LibrariesVec = Vector<std::unique_ptr<AssetLibrary>>;
  Map<eAssetLibraryType, LibrariesVec> libraries_by_type;

  static AssetLibraryRegistry &get_registry();
};

AssetLibraryRegistry &AssetLibraryRegistry::get_registry()
{
  static AssetLibraryRegistry instance;
  return instance;
}

void register_library(const eAssetLibraryType type, std::unique_ptr<AssetLibrary> library)
{
  AssetLibraryRegistry &registry = AssetLibraryRegistry::get_registry();
  registry.libraries_by_type.add_or_modify(
      type,
      [&](AssetLibraryRegistry::LibrariesVec *libraries) {
        *libraries = Vector<std::unique_ptr<AssetLibrary>>{};
        libraries->append(std::move(library));
      },
      [&](AssetLibraryRegistry::LibrariesVec *libraries) {
        libraries->append(std::move(library));
      });
}

Vector<std::unique_ptr<AssetLibrary>> unregister_libraries_of_type(const eAssetLibraryType type)
{
  AssetLibraryRegistry &registry = AssetLibraryRegistry::get_registry();
  std::optional<AssetLibraryRegistry::LibrariesVec> libraries = registry.libraries_by_type.pop_try(
      type);

  if (!libraries) {
    CLOG_INFO(&LOG, 2, "No asset libraries registered for this type [%i].", type);
  }

  return libraries;
}

void register_builtin_libraries()
{
  register_library(ASSET_LIBRARY_ALL, std::make_unique<AssetLibrary>(ASSET_LIBRARY_ALL));

  register_library(ASSET_LIBRARY_LOCAL, std::make_unique<AssetLibrary>(ASSET_LIBRARY_LOCAL));

  std::unique_ptr essentials_library = std::make_unique<AssetLibrary>(
      ASSET_LIBRARY_ESSENTIALS, "", essentials_directory_path());
  essentials_library->import_method_ = ASSET_IMPORT_APPEND_REUSE;
  register_library(ASSET_LIBRARY_ESSENTIALS, std::move(essentials_library));
}

void unregister_all_libraries()
{
  AssetLibraryRegistry &registry = AssetLibraryRegistry::get_registry();
  registry.libraries_by_type.clear_and_shrink();
}

void register_userdef_libraries(const UserDef *userdef)
{
  LISTBASE_FOREACH (bUserAssetLibrary *, user_library, &userdef->asset_libraries) {
    std::unique_ptr library = std::make_unique<AssetLibrary>(
        ASSET_LIBRARY_CUSTOM, user_library->name, user_library->dirpath);

    library->import_method_ = eAssetImportMethod(user_library->import_method);
    library->may_override_import_method_ = true;
    library->use_relative_path_ = (user_library->flag & ASSET_LIBRARY_RELATIVE_PATH) != 0;

    register_library(ASSET_LIBRARY_CUSTOM, std::move(library));
  }
}

void update_registered_userdef_libraries(const UserDef *userdef)
{
  AssetLibraryRegistry &registry = AssetLibraryRegistry::get_registry();

  Map<StringRef, int> registered_libraries_to_index;
  AssetLibraryRegistry::LibrariesVec &user_libraries = registry.libraries_by_type.lookup_default(
      ASSET_LIBRARY_CUSTOM, {});
  for (std::unique_ptr<AssetLibrary> &user_library : user_libraries) {
    //    registered_libraries_to_index.add(user_library.)
  }

  LISTBASE_FOREACH (bUserAssetLibrary *, user_library, &userdef->asset_libraries) {
  }
}

}  // namespace blender::asset_system
