/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

namespace blender {

template<typename T> struct Bounds {
  T min;
  T max;
  Bounds() = default;
  Bounds(const T &value) : min(value), max(value) {}
  Bounds(const T &min, const T &max) : min(min), max(max) {}

  bool is_empty() const;
  T center() const;
  T size() const;

  void translate(const T &offset);
  void scale_from_center(const T &scale);

  void resize(const T &new_size);
  void recenter(const T &new_center);

  template<typename PaddingT> void pad(const PaddingT &padding);
};

}  // namespace blender
