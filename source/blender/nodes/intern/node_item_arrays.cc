/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "RNA_prototypes.h"

#include "NOD_item_arrays.hh"

#include "BKE_node.hh"

namespace blender::nodes::item_arrays {

StructRNA *SimulationItemsAccessors::srna = &RNA_SimulationStateItem;
int SimulationItemsAccessors::node_type = GEO_NODE_SIMULATION_OUTPUT;

StructRNA *RepeatItemsAccessors::srna = &RNA_RepeatItem;
int RepeatItemsAccessors::node_type = GEO_NODE_REPEAT_OUTPUT;

}  // namespace blender::nodes::item_arrays
