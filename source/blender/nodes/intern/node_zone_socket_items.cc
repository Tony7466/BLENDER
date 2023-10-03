/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "RNA_prototypes.h"

#include "NOD_zone_socket_items.hh"

#include "BKE_node.hh"

namespace blender::nodes {

StructRNA *SimulationItemsAccessor::srna = &RNA_SimulationStateItem;
int SimulationItemsAccessor::node_type = GEO_NODE_SIMULATION_OUTPUT;

StructRNA *RepeatItemsAccessor::srna = &RNA_RepeatItem;
int RepeatItemsAccessor::node_type = GEO_NODE_REPEAT_OUTPUT;

}  // namespace blender::nodes
