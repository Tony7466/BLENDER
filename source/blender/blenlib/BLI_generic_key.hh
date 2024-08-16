/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <memory>

#include "BLI_utildefines.h"

namespace blender {

class GenericKey {
 public:
  virtual ~GenericKey() = default;

  virtual uint64_t hash() const = 0;

  virtual bool equal_to(const GenericKey &other) const = 0;

  virtual std::unique_ptr<GenericKey> to_storable() const = 0;

  friend bool operator==(const GenericKey &a, const GenericKey &b)
  {
    const bool are_equal = a.equal_to(b);
    /* Ensure that equality check is symmetric. */
    BLI_assert(are_equal == b.equal_to(a));
    return are_equal;
  }

  friend bool operator!=(const GenericKey &a, const GenericKey &b)
  {
    return !(a == b);
  }
};

}  // namespace blender
