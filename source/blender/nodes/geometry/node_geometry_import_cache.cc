/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_import_cache.hh"

namespace blender::nodes::geometry_import_cache {

class GeometryReadKey : public GenericKey {
 public:
  std::string file_path;

  uint64_t hash() const override
  {
    return get_default_hash(this->file_path);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_1(GeometryReadKey, file_path)

  bool equal_to(const GenericKey &other) const override
  {
    if (const auto *other_typed = dynamic_cast<const GeometryReadKey *>(&other)) {
      return *this == *other_typed;
    }
    return false;
  }

  std::unique_ptr<GenericKey> to_storable() const override
  {
    return std::make_unique<GeometryReadKey>(*this);
  }
};

void import_geometry_cache_clear_all()
{
  memory_cache::clear();
}

bke::GeometrySet import_geometry_cached(
    std::string key, FunctionRef<std::unique_ptr<GeometryReadValue>()> compute_fn)
{
  GeometryReadKey cache_key;
  cache_key.file_path = key;

  std::shared_ptr<const GeometryReadValue> value = memory_cache::get<GeometryReadValue>(
      std::move(cache_key), compute_fn);

  return value->geometry;
}

}  // namespace blender::nodes::geometry_import_cache
