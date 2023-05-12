/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 */

#include "BLI_cpp_type.hh"
#include "BLI_generic_pointer.hh"

namespace blender::nodes {

struct GeometryNodesLazyFunctionGraphInfo;

class Closure {
 private:
  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info_ = nullptr;
  Vector<GMutablePointer> bound_values_;

 public:
  Closure() = default;
  Closure(const Closure &other) = default;
  explicit Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info,
                   MutableSpan<GMutablePointer> &&bound_values);
  ~Closure();

  Closure &operator=(const Closure &other) = default;

  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info() const;

  Span<GMutablePointer> bound_values() const;
  MutableSpan<GMutablePointer> bound_values();
};

}  // namespace blender::nodes
