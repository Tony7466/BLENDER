/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <iterator>

namespace blender::iterator {

template<typename Derived> class RandomAccessIteratorMixin {
 public:
  using iterator_category = std::random_access_iterator_tag;
  using difference_type = std::ptrdiff_t;

  constexpr friend Derived &operator++(Derived &a)
  {
    ++a.iter_prop();
    return a;
  }

  constexpr friend Derived operator++(Derived &a, int)
  {
    Derived copy = a;
    ++a;
    return copy;
  }

  constexpr friend Derived &operator--(Derived &a)
  {
    --a.iter_prop();
    return a;
  }

  constexpr friend Derived operator--(Derived &a, int)
  {
    Derived copy = a;
    --a;
    return copy;
  }

  constexpr friend Derived &operator+=(Derived &a, const std::ptrdiff_t n)
  {
    a.iter_prop() += n;
    return a;
  }

  constexpr friend Derived &operator-=(Derived &a, const std::ptrdiff_t n)
  {
    a.iter_prop() -= n;
    return a;
  }

  constexpr friend Derived &operator+(const Derived &a, const std::ptrdiff_t n)
  {
    Derived copy = a;
    copy.iter_prop() += n;
    return copy;
  }

  constexpr friend Derived &operator-(const Derived &a, const std::ptrdiff_t n)
  {
    Derived copy = a;
    copy.iter_prop() -= n;
    return copy;
  }

  constexpr friend auto operator-(const Derived &a, const Derived &b)
  {
    return a.iter_prop() - b.iter_prop();
  }

  constexpr friend bool operator!=(const Derived &a, const Derived &b)
  {
    return a.iter_prop() != b.iter_prop();
  }

  constexpr friend bool operator==(const Derived &a, const Derived &b)
  {
    return a.iter_prop() == b.iter_prop();
  }

  constexpr friend bool operator<(const Derived &a, const Derived &b)
  {
    return a.iter_prop() < b.iter_prop();
  }

  constexpr friend bool operator>(const Derived &a, const Derived &b)
  {
    return a.iter_prop() > b.iter_prop();
  }

  constexpr friend bool operator<=(const Derived &a, const Derived &b)
  {
    return a.iter_prop() <= b.iter_prop();
  }

  constexpr friend bool operator>=(const Derived &a, const Derived &b)
  {
    return a.iter_prop() >= b.iter_prop();
  }

  constexpr decltype(auto) operator[](const std::ptrdiff_t i)
  {
    return *(*static_cast<Derived *>(this) + i);
  }

  constexpr decltype(auto) operator[](const std::ptrdiff_t i) const
  {
    return *(*static_cast<const Derived *>(this) + i);
  }
};

}  // namespace blender::iterator
