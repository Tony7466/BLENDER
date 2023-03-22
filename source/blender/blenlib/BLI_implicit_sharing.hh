/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <atomic>

#include "BLI_compiler_attrs.h"
#include "BLI_utildefines.h"
#include "BLI_utility_mixins.hh"

namespace blender {

/**
 * #bCopyOnWrite allows implementing copy-on-write behavior, i.e. it allows sharing read-only data
 * between multiple independend systems (e.g. meshes). The data is only copied when it is shared
 * and is about to be modified. This is in contrast to making copies before it is actually known
 * that it is necessary.
 *
 * Internally, this is mostly just a glorified reference count. If the reference count is 1, the
 * data only has a single owner and is mutable. If it is larger than 1, it is shared and must be
 * logically const.
 *
 * On top of containing a reference count, #bCopyOnWrite also knows how to destruct the referenced
 * data. This is important because the code freeing the data in the end might not know how it was
 * allocated (for example, it doesn't know whether an array was allocated using the system or
 * guarded allocator).
 *
 * #bCopyOnWrite is used in two ways:
 * - It can be allocated separately from the referenced data as is typically the case with raw
 *   arrays (e.g. for mesh attributes).
 * - It can be embedded into another struct. For that it's best to use #ImplicitShareMixin.
 */
struct ImplicitShareInfo : blender::NonCopyable, blender::NonMovable {
 private:
  mutable std::atomic<int> users_;

 public:
  ImplicitShareInfo(const int initial_users) : users_(initial_users)
  {
  }

  virtual ~ImplicitShareInfo()
  {
    BLI_assert(this->is_mutable());
  }

  bool is_shared() const
  {
    return users_.load(std::memory_order_relaxed) >= 2;
  }

  bool is_mutable() const
  {
    return !this->is_shared();
  }

  void add_user() const
  {
    users_.fetch_add(1, std::memory_order_relaxed);
  }

  void remove_user_and_delete_if_last() const
  {
    const int old_user_count = users_.fetch_sub(1, std::memory_order_relaxed);
    BLI_assert(old_user_count >= 1);
    const bool was_last_user = old_user_count == 1;
    if (was_last_user) {
      const_cast<ImplicitShareInfo *>(this)->delete_self_with_data();
    }
  }

 private:
  /** Has to free the #bCopyOnWrite and the referenced data. */
  virtual void delete_self_with_data() = 0;
};

/**
 * Makes it easy to embed copy-on-write behavior into a struct.
 */
struct ImplicitShareMixin : public ImplicitShareInfo {
 public:
  ImplicitShareMixin() : ImplicitShareInfo(1)
  {
  }

 private:
  void delete_self_with_data() override
  {
    /* Can't use `delete this` here, because we don't know what allocator was used. */
    this->delete_self();
  }

  virtual void delete_self() = 0;
};

}  // namespace blender
