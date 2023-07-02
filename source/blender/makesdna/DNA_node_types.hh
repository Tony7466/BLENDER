/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_sys_types.h"

namespace blender::dna {

struct NodeGeometryUVUnwrap {
  /** #GeometryNodeUVUnwrapMethod. */
  uint8_t method;
};

typedef struct NodeSimulationItem {
  char *name;
  /** #eNodeSocketDatatype. */
  short socket_type;
  /** #eAttrDomain. */
  short attribute_domain;
  /**
   * Generates unique identifier for sockets which stays the same even when the item order or
   * names change.
   */
  int identifier;
} NodeSimulationItem;

typedef struct NodeGeometrySimulationInput {
  /** bNode.identifier of the corresponding output node. */
  int32_t output_node_id;
} NodeGeometrySimulationInput;

typedef struct NodeGeometrySimulationOutput {
  NodeSimulationItem *items;
  int items_num;
  int active_index;
  /** Number to give unique IDs to state items. */
  int next_identifier;
  int _pad;

#ifdef __cplusplus
  blender::Span<NodeSimulationItem> items_span() const;
  blender::MutableSpan<NodeSimulationItem> items_span();
  blender::IndexRange items_range() const;
#endif
} NodeGeometrySimulationOutput;

}  // namespace blender::dna
