/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "NOD_closure.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender::nodes {

Closure::Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info,
                 MutableSpan<GMutablePointer> &&bound_values)
    : lf_graph_info_(&lf_graph_info), bound_values_(std::move(bound_values))
{
}

Closure::~Closure() {}

const GeometryNodesLazyFunctionGraphInfo *Closure::lf_graph_info() const
{
  return lf_graph_info_;
}

Span<GMutablePointer> Closure::bound_values() const
{
  return bound_values_;
}

MutableSpan<GMutablePointer> Closure::bound_values()
{
  return bound_values_;
}

}  // namespace blender::nodes
