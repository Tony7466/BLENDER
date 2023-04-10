/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup fn
 */

#include "BLI_cpp_type.hh"

#include "FN_lazy_function.hh"
#include "FN_lazy_function_graph_executor.hh"

namespace blender::fn {

class Closure {
 private:
  std::shared_ptr<lf::GraphExecutor> function_;

 public:
  Closure() = default;
  Closure(const Closure &other) = default;
  explicit Closure(const lf::GraphExecutor &function);
  ~Closure() = default;

  Closure &operator=(const Closure &other) = default;
};


}  // namespace blender::fn
