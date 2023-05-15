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
  const blender::nodes::GeometryNodesLazyFunctionGraphInfo *lf_graph_info =
      ensure_geometry_nodes_lazy_function_graph(*node_tree);
  BLI_assert(lf_graph_info);

  Array<GMutablePointer> bound_values(node_tree->interface_inputs().size());
  for (const int i : node_tree->interface_inputs().index_range()) {
    const bNodeSocket *socket = node_tree->interface_inputs()[i];
    const CPPType *cpptype = socket->typeinfo->geometry_nodes_cpp_type;
    if (cpptype && socket->typeinfo->get_geometry_nodes_cpp_value) {
      void *buffer = MEM_mallocN_aligned(
          cpptype->size(), cpptype->alignment(), "default graph input buffer");
      socket->typeinfo->get_geometry_nodes_cpp_value(*socket, buffer);
      bound_values[i] = {cpptype, buffer};
    }
  }

  return Closure(*lf_graph_info, bound_values);
}

}  // namespace blender::nodes
