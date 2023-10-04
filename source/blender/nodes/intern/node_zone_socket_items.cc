/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "RNA_prototypes.h"

#include "NOD_zone_socket_items.hh"

#include "BKE_node.hh"

#include "BLO_read_write.hh"

namespace blender::nodes {

/* Defined here to avoid including the relevant headers in the header. */

StructRNA *SimulationItemsAccessor::item_srna = &RNA_SimulationStateItem;
int SimulationItemsAccessor::node_type = GEO_NODE_SIMULATION_OUTPUT;

void SimulationItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometrySimulationOutput *>(node.storage);
  BLO_write_struct_array(writer, NodeSimulationItem, storage.items_num, storage.items);
  for (const NodeSimulationItem &item : Span(storage.items, storage.items_num)) {
    BLO_write_string(writer, item.name);
  }
}

void SimulationItemsAccessor::blend_read_data(BlendDataReader *reader, bNode &node)
{
  auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
  BLO_read_data_address(reader, &storage.items);
  for (const NodeSimulationItem &item : Span(storage.items, storage.items_num)) {
    BLO_read_data_address(reader, &item.name);
  }
}

StructRNA *RepeatItemsAccessor::item_srna = &RNA_RepeatItem;
int RepeatItemsAccessor::node_type = GEO_NODE_REPEAT_OUTPUT;

void RepeatItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryRepeatOutput *>(node.storage);
  BLO_write_struct_array(writer, NodeRepeatItem, storage.items_num, storage.items);
  for (const NodeRepeatItem &item : Span(storage.items, storage.items_num)) {
    BLO_write_string(writer, item.name);
  }
}

void RepeatItemsAccessor::blend_read_data(BlendDataReader *reader, bNode &node)
{
  auto &storage = *static_cast<NodeGeometryRepeatOutput *>(node.storage);
  BLO_read_data_address(reader, &storage.items);
  for (const NodeRepeatItem &item : Span(storage.items, storage.items_num)) {
    BLO_read_data_address(reader, &item.name);
  }
}

StructRNA *ForEachInputItemsAccessor::item_srna = &RNA_ForEachInputItem;
int ForEachInputItemsAccessor::node_type = GEO_NODE_FOR_EACH_OUTPUT;

void ForEachInputItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryForEachOutput *>(node.storage);
  BLO_write_struct_array(
      writer, NodeForEachInputItem, storage.input_items_num, storage.input_items);
  for (const NodeForEachInputItem &item : Span(storage.input_items, storage.input_items_num)) {
    BLO_write_string(writer, item.name);
  }
}

void ForEachInputItemsAccessor::blend_read_data(BlendDataReader *reader, bNode &node)
{
  auto &storage = *static_cast<NodeGeometryForEachOutput *>(node.storage);
  BLO_read_data_address(reader, &storage.input_items);
  for (const NodeForEachInputItem &item : Span(storage.input_items, storage.input_items_num)) {
    BLO_read_data_address(reader, &item.name);
  }
}

StructRNA *ForEachOutputItemsAccessor::item_srna = &RNA_ForEachOutputItem;
int ForEachOutputItemsAccessor::node_type = GEO_NODE_REPEAT_OUTPUT;

void ForEachOutputItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryForEachOutput *>(node.storage);
  BLO_write_struct_array(
      writer, NodeForEachOutputItem, storage.output_items_num, storage.output_items);
  for (const NodeForEachOutputItem &item : Span(storage.output_items, storage.output_items_num)) {
    BLO_write_string(writer, item.name);
  }
}

void ForEachOutputItemsAccessor::blend_read_data(BlendDataReader *reader, bNode &node)
{
  auto &storage = *static_cast<NodeGeometryForEachOutput *>(node.storage);
  BLO_read_data_address(reader, &storage.output_items);
  for (const NodeForEachOutputItem &item : Span(storage.output_items, storage.output_items_num)) {
    BLO_read_data_address(reader, &item.name);
  }
}

}  // namespace blender::nodes
