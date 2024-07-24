/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

/** \file
 * \ingroup intern_memutil
 *
 * Implement a static storage for complex, non-static allocation strings passed MEM_guardedalloc
 * functions.
 */

#include <any>
#include <cassert>
#include <string>
#include <unordered_map>

namespace intern::memutil {

template<typename keyT, template<typename> typename hashT> class AllocStringStorage {
  std::unordered_map<keyT, std::string, hashT<keyT>> storage_;

 public:
  /**
   * Check whether the given key exists in the specified storage.
   *
   * \param storage_identifier: String identifier for a given storage.
   * \return `true` if the \a key is found in specified storage, false otherwise.
   */
  bool contains(keyT &key)
  {
    return storage_.count(key) != 0;
  }

  /**
   * Return the alloc string for the given key in the specified storage.
   *
   * \param storage_identifier: String identifier for a given storage.
   * \return A pointer to the stored string if \a key is found, `nullptr` otherwise.
   */
  const char *find(keyT &key)
  {
    if (storage_.count(key) != 0) {
      return storage_[key].c_str();
    }
    return nullptr;
  }

  /**
   * Insert the given alloc string in the specified storage, at the given key, and return a pointer
   * to the stored string.
   *
   * \param storage_identifier: String identifier for a given storage.
   * \param alloc_string: The alloc string to store at \a key.
   * \return A pointer to the inserted stored string.
   */
  const char *insert(keyT &key, std::string alloc_string)
  {
#ifndef NDEBUG
    assert(storage_.count(key) == 0);
#endif
    return (storage_[key] = alloc_string).c_str();
  }
};

namespace internal {
/**
 * @brief The AllocStringStorage class
 */
class AllocStringStorageContainer {
  std::unordered_map<std::string, std::any> storage_;

 public:
  /** Create or return an existing mapping for the given storage identifier. */
  template<typename keyT, template<typename> typename hashT>
  std::any &ensure_storage(const std::string &storage_identifier)
  {
    if (storage_.count(storage_identifier) == 0) {
      AllocStringStorage<keyT, hashT> storage_for_identifier;
      return (storage_[storage_identifier] = std::make_any<AllocStringStorage<keyT, hashT>>(
                  std::move(storage_for_identifier)));
    }
    return storage_[storage_identifier];
  }
};

AllocStringStorageContainer &ensure_storage_container();

}  // namespace internal

/**
 * Initialize the static storage for MEM_guardedalloc allocation strings.
 *
 * Must be called before #MEM_init_memleak_detection, to ensure that it is destroyed after the
 * memkleak has run and been destroyed. Otherwise, printing allocation strings of unfreed (leaking)
 * memblocks could access freed memory.
 */
void alloc_string_storage_init();

/**
 * Get a reference to the AllocStringStorage static data.
 */
template<typename keyT, template<typename> typename hashT>
AllocStringStorage<keyT, hashT> &alloc_string_storage_get(const std::string &storage_identifier)
{
  internal::AllocStringStorageContainer &storage_container = internal::ensure_storage_container();
  std::any &storage = storage_container.ensure_storage<keyT, hashT>(storage_identifier);
  return std::any_cast<AllocStringStorage<keyT, hashT> &>(storage);
}

}  // namespace intern::memutil
