/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_inverse_eval.hh"

#include "BKE_node_socket_value.hh"

#include "BLI_compute_context.hh"

#include "NOD_geometry_nodes_log.hh"

struct NodesModifierData;

namespace blender::nodes::inverse_eval {

using bke::SocketValueVariant;

bool try_change_link_target_and_update_source(bContext &C,
                                              Object &object,
                                              NodesModifierData &nmd,
                                              geo_eval_log::GeoModifierLog &eval_log,
                                              const ComputeContext *initial_context,
                                              const bNodeLink &initial_link,
                                              const SocketValueVariant &new_value);

std::optional<SocketValueVariant> get_logged_socket_value(geo_eval_log::GeoTreeLog &tree_log,
                                                          const bNodeSocket &socket);

std::optional<bke::SocketValueVariant> convert_socket_value(
    const bNodeSocket &old_socket,
    const bNodeSocket &new_socket,
    const bke::SocketValueVariant &old_value);

}  // namespace blender::nodes::inverse_eval
