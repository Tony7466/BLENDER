/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup fn
 */

#include "BLI_cpp_type.hh"

namespace blender::fn {

namespace lazy_function {
class Graph;
}

class Closure {
 private:
  const lazy_function::Graph *graph_;

 public:
  Closure() = default;
  Closure(const Closure &other) = default;
  explicit Closure(const lazy_function::Graph &graph);
  ~Closure() = default;

  Closure &operator=(const Closure &other) = default;
};


}  // namespace blender::fn
