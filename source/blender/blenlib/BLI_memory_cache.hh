/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_generic_key.hh"
#include "BLI_memory_counter_fwd.hh"

namespace blender::memory_cache {

/**
 * A value that is stored in the cache. It may be freed automatically when the cache is full.
 */
class CachedValue {
 public:
  virtual ~CachedValue() = default;

  /**
   * Gather the memory used by this value. This allows the cache system to determine when it is
   * full.
   */
  virtual void count_memory(MemoryCounter &memory) const = 0;
};

/**
 * Returns the value that corresponds to the given key. If it's not cached yet, #compute_fn is
 * called and its result is cached for the next time.
 *
 * If the cache is full, older values may be freed.
 */
std::shared_ptr<CachedValue> get_base(const GenericKey &key,
                                      FunctionRef<std::unique_ptr<CachedValue>()> compute_fn);

/**
 * Same as above, but with an additional type-cast to simplify the caller code.
 */
template<typename T>
inline std::shared_ptr<const T> get(const GenericKey &key,
                                    FunctionRef<std::unique_ptr<T>()> compute_fn)
{
  return std::dynamic_pointer_cast<const T>(get_base(key, compute_fn));
}

/**
 * Set how much memory the cache is allows to use. This is only an approximation because counting
 * the memory is not 100% accurate, and for some types the memory usage may even change over time.
 */
void set_approximate_size_limit(int64_t limit_in_bytes);

}  // namespace blender::memory_cache
