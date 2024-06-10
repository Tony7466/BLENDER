/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_inverse_eval.hh"

#include "BKE_node_socket_value.hh"

#include "BLI_compute_context.hh"

namespace blender::nodes::inverse_eval {

using bke::SocketValueVariant;

void change_socket_value_and_update_source(
    Object &object,
    const ComputeContext &context,
    bNodeSocket &socket,
    FunctionRef<SocketValueVariant(SocketValueVariant old_value)> update_fn);

}  // namespace blender::nodes::inverse_eval
