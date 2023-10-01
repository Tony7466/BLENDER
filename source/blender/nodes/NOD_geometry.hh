/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node.h"

extern bNodeTreeType *ntreeType_Geometry;

void register_node_tree_type_geo();
void register_node_type_geo_custom_group(bNodeType *ntype);

/* -------------------------------------------------------------------- */
/** \name Simulation Output Node
 * \{ */

bool NOD_geometry_simulation_output_item_socket_type_supported(eNodeSocketDatatype socket_type);

/**
 * Set a unique item name.
 * \return True if the unique name differs from the original name.
 */
bool NOD_geometry_simulation_output_item_set_unique_name(NodeGeometrySimulationOutput *sim,
                                                         NodeSimulationItem *item,
                                                         const char *name,
                                                         const char *defname);

/**
 * Find the node owning this simulation state item.
 */
NodeSimulationItem *NOD_geometry_simulation_output_insert_item(NodeGeometrySimulationOutput *sim,
                                                               short socket_type,
                                                               const char *name,
                                                               int index);
NodeSimulationItem *NOD_geometry_simulation_output_add_item_from_socket(
    NodeGeometrySimulationOutput *sim, const bNode *from_node, const bNodeSocket *from_sock);

/** \} */
