/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "NOD_closure.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender::nodes {

Closure::Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
    : lf_graph_info_(&lf_graph_info)
{
}

const GeometryNodesLazyFunctionGraphInfo *Closure::lf_graph_info() const
{
  return *lf_graph_info_;
}

}  // namespace blender::nodes
