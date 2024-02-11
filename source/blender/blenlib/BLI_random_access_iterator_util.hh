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
    derived.get_prop()++;
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
    derived.get_prop()--;
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
    a.get_prop() += n;
    return a.as_derived();
  }

  constexpr friend Derived &operator-=(RandomAccessIteratorMixin &a, const int64_t n)
  {
    a.get_prop() -= n;
    return a.as_derived();
  }

  constexpr friend auto operator-(const RandomAccessIteratorMixin &a,
                                  const RandomAccessIteratorMixin &b)
  {
    return a.get_prop() - b.get_prop();
  }

  constexpr friend bool operator!=(const RandomAccessIteratorMixin &a,
                                   const RandomAccessIteratorMixin &b)
  {
    return a.get_prop() != b.get_prop();
  }

  constexpr friend bool operator==(const RandomAccessIteratorMixin &a,
                                   const RandomAccessIteratorMixin &b)
  {
    return a.get_prop() == b.get_prop();
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

  auto &get_prop()
  {
    return this->as_derived().get_property();
  }

  const auto &get_prop() const
  {
    return this->as_derived().get_property();
  }
};

}  // namespace blender::iterator
