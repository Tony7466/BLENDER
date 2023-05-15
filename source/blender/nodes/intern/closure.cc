/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "NOD_closure.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender::nodes {

ClosureInputValues::ClosureInputValues(const Span<GMutablePointer> &values) : values_(values) {}

ClosureInputValues::~ClosureInputValues()
{
  for (GMutablePointer &ptr : values_) {
    ptr.destruct();
    MEM_freeN(ptr.get());
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

Closure::Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info,
                 MutableSpan<GMutablePointer> &&bound_values)
    : lf_graph_info_(&lf_graph_info),
      bound_values_(std::make_shared<ClosureInputValues>(bound_values))
{
}

Closure::~Closure() {}

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

}  // namespace blender::nodes
