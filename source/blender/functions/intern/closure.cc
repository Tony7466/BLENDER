/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "FN_closure.hh"

#include "FN_lazy_function_graph.hh"

namespace blender::fn {

Closure::Closure(const lf::Graph &graph)
    : graph_(&graph)
{
}

}  // namespace blender::fn
