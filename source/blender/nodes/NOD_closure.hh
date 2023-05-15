/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 */

#include "BLI_generic_pointer.hh"

struct bNodeTree;

namespace blender::nodes {

struct GeometryNodesLazyFunctionGraphInfo;

class ClosureInputValues {
 private:
  Vector<GMutablePointer> values_;

 public:
  ClosureInputValues(const Span<GMutablePointer> &values);
  ~ClosureInputValues();

  Span<GMutablePointer> values() const;
  MutableSpan<GMutablePointer> values();
};

class Closure {
 private:
  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info_ = nullptr;
  std::shared_ptr<ClosureInputValues> bound_values_;

 public:
  Closure();
  Closure(const Closure &other);
  explicit Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info,
                   MutableSpan<GMutablePointer> &&bound_values);
  ~Closure() = default;

  Closure &operator=(const Closure &other) = default;

  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info() const;

  Span<GMutablePointer> bound_values() const;
  MutableSpan<GMutablePointer> bound_values();

  static Closure make_from_node_tree(const bNodeTree *node_tree);
};

}  // namespace blender::nodes
