/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_bounds_types.hh"
#include "BLI_implicit_sharing_cache.hh"
#include "BLI_virtual_array.hh"

namespace blender::bounds {

template<typename T> inline auto &get_bounds_cache()
{
  static implicit_sharing::Cache<Bounds<T>> cache;
  return cache;
}

template<typename T>
inline std::optional<Bounds<T>> min_max(const Span<T> values,
                                        const ImplicitSharingInfo *sharing_info)
{
  if (values.is_empty()) {
    return std::nullopt;
  }
  if (sharing_info == nullptr) {
    return min_max(values);
  }
  implicit_sharing::Cache<Bounds<T>> &cache = get_bounds_cache<T>();
  implicit_sharing::CacheKey cache_key;
  cache_key.inputs.append_as(*sharing_info, values.data());
  return cache.lookup_or_compute([&]() { return *min_max(values); }, cache_key);
}

template<typename T>
inline void set_min_max(const Span<T> values,
                        const ImplicitSharingInfo *sharing_info,
                        const Bounds<T> &bounds)
{
  if (sharing_info == nullptr) {
    return;
  }
  implicit_sharing::Cache<Bounds<T>> &cache = get_bounds_cache<T>();
  implicit_sharing::CacheKey cache_key;
  cache_key.inputs.append_as(*sharing_info, values.data());
  cache.lookup_or_compute([&]() { return bounds; }, cache_key);
}

}  // namespace blender::bounds
