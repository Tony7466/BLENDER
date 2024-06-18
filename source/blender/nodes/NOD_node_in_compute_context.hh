/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_compute_context.hh"
#include "BLI_hash.hh"
#include "BLI_struct_equality_utils.hh"

struct bNode;
struct bNodeSocket;

namespace blender::nodes {

struct NodeInContext {
  const ComputeContext *context = nullptr;
  const bNode *node = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context_hash(), this->node);
  }

  ComputeContextHash context_hash() const
  {
    return context ? context->hash() : ComputeContextHash{};
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(NodeInContext, context_hash(), node)
};

struct SocketInContext {
  const ComputeContext *context = nullptr;
  const bNodeSocket *socket = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context_hash(), this->socket);
  }

  ComputeContextHash context_hash() const
  {
    return context ? context->hash() : ComputeContextHash{};
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(SocketInContext, context_hash(), socket)
};

}  // namespace blender::nodes
