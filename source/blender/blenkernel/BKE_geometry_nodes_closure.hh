/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_nodes_bundle.hh"
#include "BKE_geometry_nodes_closure_fwd.hh"

#include "FN_lazy_function.hh"

namespace blender::bke {

class ClosureSignature {
 private:
  std::shared_ptr<SocketListSignature> inputs_sockets_;
  std::shared_ptr<SocketListSignature> outputs_sockets_;

 public:
  ClosureSignature(std::shared_ptr<SocketListSignature> input_sockets,
                   std::shared_ptr<SocketListSignature> output_sockets)
      : inputs_sockets_(input_sockets), outputs_sockets_(output_sockets)
  {
  }

  const SocketListSignature &inputs_sockets() const
  {
    return *inputs_sockets_;
  }
  const SocketListSignature &outputs_sockets() const
  {
    return *outputs_sockets_;
  }
};

class Closure : public ImplicitSharingInfo {
 private:
  std::shared_ptr<ClosureSignature> signature_;
  fn::lazy_function::LazyFunction &function_;

 public:
  Closure(std::shared_ptr<ClosureSignature> signature, fn::lazy_function::LazyFunction &function)
      : signature_(signature), function_(function)
  {
  }

  ClosureSignature &signature() const
  {
    return *signature_;
  }

  fn::lazy_function::LazyFunction &function() const
  {
    return function_;
  }

  void delete_self_with_data() override
  {
    MEM_delete(this);
  }
};

}  // namespace blender::bke
