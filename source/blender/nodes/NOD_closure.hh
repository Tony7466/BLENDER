/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 */

#include "BLI_cpp_type.hh"

namespace blender::nodes {

struct GeometryNodesLazyFunctionGraphInfo;

class Closure {
 private:
  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info_;

 public:
  Closure() = default;
  Closure(const Closure &other) = default;
  explicit Closure(const GeometryNodesLazyFunctionGraphInfo &lf_graph_info);
  ~Closure() = default;

  Closure &operator=(const Closure &other) = default;

  const GeometryNodesLazyFunctionGraphInfo *lf_graph_info() const;
};


}  // namespace blender::nodes
