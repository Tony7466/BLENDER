/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup fn
 */

#include "FN_closure.hh"

namespace blender::fn {

Closure::Closure(const lf::GraphExecutor &function)
    : function_(std::make_shared<lf::GraphExecutor>(function))
{
}

}  // namespace blender::fn
