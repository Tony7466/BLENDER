/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <iterator>

namespace blender::iterator {

template<typename Derived> class RandomAccessIteratorMixin {
 public:
  using iterator_category = std::random_access_iterator_tag;

  constexpr Derived &operator++()
  {
    auto &derived = this->as_derived();
    derived.iter_prop()++;
    return derived;
  }

  constexpr Derived operator++(int)
  {
    Derived copied = this->as_derived();
    ++(*this);
    return copied;
  }

  constexpr Derived &operator--()
  {
    auto &derived = this->as_derived();
    derived.iter_prop()--;
    return derived;
  }

  constexpr Derived operator--(int)
  {
    Derived copied = this->as_derived();
    --(*this);
    return copied;
  }

  constexpr friend Derived &operator+=(RandomAccessIteratorMixin &a, const int64_t n)
  {
    a.iter_prop() += n;
    return a.as_derived();
  }

  constexpr friend Derived &operator-=(RandomAccessIteratorMixin &a, const int64_t n)
  {
    a.iter_prop() -= n;
    return a.as_derived();
  }

  constexpr friend auto operator-(const RandomAccessIteratorMixin &a,
                                  const RandomAccessIteratorMixin &b)
  {
    return a.iter_prop() - b.iter_prop();
  }

  constexpr friend bool operator!=(const RandomAccessIteratorMixin &a,
                                   const RandomAccessIteratorMixin &b)
  {
    return a.iter_prop() != b.iter_prop();
  }

  constexpr friend bool operator==(const RandomAccessIteratorMixin &a,
                                   const RandomAccessIteratorMixin &b)
  {
    return a.iter_prop() == b.iter_prop();
  }

 private:
  Derived &as_derived()
  {
    return *static_cast<Derived *>(this);
  }

  const Derived &as_derived() const
  {
    return *static_cast<const Derived *>(this);
  }

  auto &iter_prop()
  {
    return this->as_derived().iter_prop();
  }

  const auto &iter_prop() const
  {
    return this->as_derived().iter_prop();
  }
};

}  // namespace blender::iterator
