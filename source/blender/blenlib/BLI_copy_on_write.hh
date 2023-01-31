/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <atomic>

#include "BLI_compiler_attrs.h"
#include "BLI_copy_on_write.h"
#include "BLI_utildefines.h"
#include "BLI_utility_mixins.hh"

struct bCopyOnWrite : blender::NonCopyable, blender::NonMovable {
 private:
  mutable std::atomic<int> users_;

 public:
  bCopyOnWrite(const int initial_users) : users_(initial_users)
  {
  }

  ~bCopyOnWrite()
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
      const_cast<bCopyOnWrite *>(this)->delete_self_with_data();
    }
  }

 private:
  virtual void delete_self_with_data() = 0;
};
