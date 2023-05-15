/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "NOD_closure.hh"

#include "DNA_node_types.h"

#include "BKE_node_runtime.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender::nodes {

ClosureInputValues::ClosureInputValues(const Span<GMutablePointer> &values) : values_(values) {}

ClosureInputValues::~ClosureInputValues()
{
  for (GMutablePointer &ptr : values_) {
    if (ptr.get()) {
      ptr.destruct();
      MEM_freeN(ptr.get());
    }
  }
}

Span<GMutablePointer> ClosureInputValues::values() const
{
  return values_;
}

MutableSpan<GMutablePointer> ClosureInputValues::values()
{
  return values_;
}

Closure::Closure() : bound_values_(nullptr) {}

Closure::Closure(const Closure &other)
    : lf_graph_info_(other.lf_graph_info_), bound_values_(other.bound_values_)
{
}

Closure::Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info,
                 MutableSpan<GMutablePointer> &&bound_values)
    : lf_graph_info_(&lf_graph_info),
      bound_values_(std::make_shared<ClosureInputValues>(bound_values))
{
}

const GeometryNodesLazyFunctionGraphInfo *Closure::lf_graph_info() const
{
  return lf_graph_info_;
}

Span<GMutablePointer> Closure::bound_values() const
{
  return bound_values_->values();
}

MutableSpan<GMutablePointer> Closure::bound_values()
{
  return bound_values_->values();
}

Closure Closure ::make_from_node_tree(const bNodeTree *node_tree)
{
  if (node_tree == nullptr) {
    return Closure();
  }

  BLI_assert(node_tree->runtime);
  const std::unique_ptr<blender::nodes::GeometryNodesLazyFunctionGraphInfo> &lf_graph_info_ptr =
      node_tree->runtime->geometry_nodes_lazy_function_graph_info;
  BLI_assert(lf_graph_info_ptr);

  Array<GMutablePointer> bound_values(node_tree->interface_inputs().size());
  // TODO fill with default values of the tree

  return Closure(*lf_graph_info_ptr.get(), bound_values);
}

}  // namespace blender::nodes
