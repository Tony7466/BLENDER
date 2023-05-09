/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <mutex>

#include "BLI_array.hh"
#include "BLI_function_ref.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_map.hh"
#include "BLI_task.hh"

namespace blender::implicit_sharing {

struct Snapshot {
  const ImplicitSharingInfo *sharing_info;
  int64_t version;
  const void *data;

  Snapshot(const ImplicitSharingInfo &sharing_info, const void *data)
      : sharing_info(&sharing_info), version(sharing_info.version()), data(data)
  {
  }

  uint64_t hash() const
  {
    return get_default_hash_3(this->sharing_info, this->version, this->data);
  }

  friend bool operator==(const Snapshot &a, const Snapshot &b)
  {
    return a.sharing_info == b.sharing_info && a.version == b.version && a.data == b.data;
  }
};

struct CacheKey {
  Vector<Snapshot> inputs;

  uint64_t hash() const
  {
    return get_default_hash(this->inputs.as_span());
  }

  friend bool operator==(const CacheKey &a, const CacheKey &b)
  {
    return a.inputs.as_span() == b.inputs.as_span();
  }
};

template<typename T> class Cache {
 private:
  std::mutex mutex_;
  Map<CacheKey, T> map_;

 public:
  ~Cache()
  {
    for (const CacheKey &key : map_.keys()) {
      for (const Snapshot &snapshot : key.inputs) {
        snapshot.sharing_info->remove_weak_user_and_delete_if_last();
      }
    }
  }

  void clear_unused()
  {
    std::lock_guard lock{mutex_};
    map_.remove_if([&](MutableMapItem<CacheKey, T> item) {
      bool found_expired_input = false;
      for (const Snapshot &snapshot : item.key.inputs) {
        if (snapshot.sharing_info->is_expired() ||
            snapshot.sharing_info->version() != snapshot.version) {
          found_expired_input = true;
        }
      }
      if (found_expired_input) {
        for (const Snapshot &snapshot : item.key.inputs) {
          snapshot.sharing_info->remove_weak_user_and_delete_if_last();
        }
      }
      return found_expired_input;
    });
  }

  T lookup_or_compute(const FunctionRef<T()> fn, const CacheKey &key)
  {
    this->clear_unused();
    std::lock_guard lock{mutex_};
    if (T *value = map_.lookup_ptr(key)) {
      return *value;
    }
    std::optional<T> new_value;
    threading::isolate_task([&]() { new_value = fn(); });
    map_.add_new(key, *new_value);
    for (const Snapshot &snapshot : key.inputs) {
      snapshot.sharing_info->add_weak_user();
    }
    return std::move(*new_value);
  }
};

}  // namespace blender::implicit_sharing
