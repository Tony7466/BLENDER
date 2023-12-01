/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#pragma once

#include <ostream>

#include "BLI_math_vector_types.hh"
#include "BLI_utildefines.h"

namespace blender {

template<typename T, int Size> struct rect_struct_base {
  VecBase<T, Size> min;
  VecBase<T, Size> max;
};

template<typename T> struct rect_struct_base<T, 2> {
  VecBase<T, 2> min;
  VecBase<T, 2> max;
};

template<typename T> struct rect_struct_base<T, 3> {
  VecBase<T, 3> min;
  VecBase<T, 3> max;
};

template<typename T, int Size> struct RectBase : rect_struct_base<T, Size> {
  using vec_base = VecBase<T, Size>;

  RectBase()
  {
    this->min = vec_base(0);
    this->max = vec_base(0);
  };

  RectBase(vec_base _min, vec_base _max)
  {
    this->min = _min;
    this->max = _max;
  }

  bool is_empty() const
  {
    return this->max <= this->min;
  }

  bool is_valid() const
  {
    return this->min <= this->max;
  }

  void sanitize()
  {
    if (!this->is_valid()) {
      std::swap(this->min, this->max);
    }
    BLI_assert(this->is_valid());
  }

  bool intersects(const vec_base &p) const
  {
    for (int i = 0; i < Size; i++) {
    }
    if (p < this->min) {
      return false;
    }
    if (p > this->max) {
      return false;
    }
    return true;
  }

  vec_base size() const
  {
    return this->max - this->min;
  }

  /** Compare. */

  friend bool operator==(const RectBase &a, const RectBase &b)
  {
    return (a.min == b.min) && (a.max == b.max);
  }

  friend bool operator!=(const RectBase &a, const RectBase &b)
  {
    return !(a == b);
  }

  /** Misc. */

  uint64_t hash() const
  {
    return math::vector_hash(*this);
  }

  friend std::ostream &operator<<(std::ostream &stream, const RectBase &r)
  {
    stream << "(min: " << r.min << ", max: " << r.max << ", size: " << r.size() << ")";
    return stream;
  }
};

using rect2f = RectBase<float, 2>;
using rect2i = RectBase<int32_t, 2>;

using rect3f = RectBase<float, 3>;
using rect3i = RectBase<int32_t, 3>;

}  // namespace blender