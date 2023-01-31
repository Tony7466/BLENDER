/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#include <string>
#include <variant>

#include "BLI_map.hh"

#include "DNA_userdef_types.h"

#include "AS_asset_library_loader.hh"

#include "AS_asset_system_registry.hh"

namespace blender::asset_system {

class AssetLibraryDescription {
  using OnDiskDescription = bUserAssetLibrary;
  struct CustomDescription {
    std::string ui_name;
    std::string identifier;

    bool operator==(const CustomDescription &other) const
    {
      return identifier == other.identifier;
    }
    uint64_t hash() const
    {
      return get_default_hash(identifier);
    }
  };

  std::variant<OnDiskDescription *, CustomDescription> description_;

 public:
  AssetLibraryDescription(bUserAssetLibrary &on_disk) : description_(&on_disk)
  {
  }

  AssetLibraryDescription(StringRef ui_name, StringRef identifier)
      : description_(CustomDescription{ui_name, identifier})
  {
  }

  bool operator==(const AssetLibraryDescription &other) const
  {
    return description_ == other.description_;
  }

  uint64_t hash() const
  {
    return std::visit([](const auto &description) { return get_default_hash(description); },
                      description_);
  }
};

class AssetSystemRegistry {
 public:
  static AssetSystemRegistry &get_instance();

  Map<AssetLibraryDescription, std::unique_ptr<AbstractAssetLibraryLoader>>
      instanceable_libraries_;
};

AssetSystemRegistry &AssetSystemRegistry::get_instance()
{
  static AssetSystemRegistry instance_;
  return instance_;
}

void register_asset_library(StringRef ui_name,
                            StringRef identifier,
                            std::unique_ptr<AbstractAssetLibraryLoader> loader)
{
  AssetSystemRegistry &registry = AssetSystemRegistry::get_instance();
  registry.instanceable_libraries_.add_overwrite({ui_name, identifier}, std::move(loader));
}

void register_asset_library(bUserAssetLibrary &on_disk_library,
                            std::unique_ptr<AbstractAssetLibraryLoader> loader)
{
  AssetSystemRegistry &registry = AssetSystemRegistry::get_instance();
  registry.instanceable_libraries_.add_overwrite(on_disk_library, std::move(loader));
}

void unregister_asset_library(StringRef identifier)
{
  AssetSystemRegistry &registry = AssetSystemRegistry::get_instance();
  /* Construct a description without name, only the identifier is needed to use the description as
   * key. */
  registry.instanceable_libraries_.remove({"", identifier});
}

void unregister_asset_library(bUserAssetLibrary &on_disk_library)
{
  AssetSystemRegistry &registry = AssetSystemRegistry::get_instance();
  registry.instanceable_libraries_.remove(on_disk_library);
}

}  // namespace blender::asset_system
