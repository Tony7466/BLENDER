/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cmath>

#include "BLI_hash.h"
#include "BLI_time.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_random_seed_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Random Seed")
      .description("A pseudo-random seed based on the current time");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (!check_tool_context_and_error(params)) {
    return;
  }
  params.set_output("Random Seed", params.user_data()->call_data->operator_data->random_seed);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_TOOL_RANDOM_SEED, "Random Seed", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = search_link_ops_for_tool_node;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_random_seed_cc
