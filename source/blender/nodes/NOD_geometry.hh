/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node.h"

extern bNodeTreeType *ntreeType_Geometry;

void register_node_type_geo_custom_group(bNodeType *ntype);

/* -------------------------------------------------------------------- */
/** \name Simulation Input Node
 * \{ */

bNode *NOD_geometry_simulation_input_get_paired_output(bNodeTree *node_tree,
                                                       const bNode *simulation_input_node);

/**
 * Pair a simulation input node with an output node.
 * \return True if pairing the node was successful.
 */
bool NOD_geometry_simulation_input_pair_with_output(const bNodeTree *node_tree,
                                                    bNode *simulation_input_node,
                                                    const bNode *simulation_output_node);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Simulation Output Node
 * \{ */

bool NOD_geometry_simulation_output_item_socket_type_supported(eNodeSocketDatatype socket_type);

/**
 * Set a unique item name.
 * \return True if the unique name differs from the original name.
 */
bool NOD_geometry_simulation_output_item_set_unique_name(
    blender::dna::NodeGeometrySimulationOutput *sim,
    blender::dna::NodeSimulationItem *item,
    const char *name,
    const char *defname);

/**
 * Find the node owning this simulation state item.
 */
bNode *NOD_geometry_simulation_output_find_node_by_item(
    bNodeTree *ntree, const blender::dna::NodeSimulationItem *item);

bool NOD_geometry_simulation_output_contains_item(blender::dna::NodeGeometrySimulationOutput *sim,
                                                  const blender::dna::NodeSimulationItem *item);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_get_active_item(
    blender::dna::NodeGeometrySimulationOutput *sim);
void NOD_geometry_simulation_output_set_active_item(
    blender::dna::NodeGeometrySimulationOutput *sim, blender::dna::NodeSimulationItem *item);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_find_item(
    blender::dna::NodeGeometrySimulationOutput *sim, const char *name);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_add_item(
    blender::dna::NodeGeometrySimulationOutput *sim, short socket_type, const char *name);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_insert_item(
    blender::dna::NodeGeometrySimulationOutput *sim,
    short socket_type,
    const char *name,
    int index);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_add_item_from_socket(
    blender::dna::NodeGeometrySimulationOutput *sim,
    const bNode *from_node,
    const bNodeSocket *from_sock);
blender::dna::NodeSimulationItem *NOD_geometry_simulation_output_insert_item_from_socket(
    blender::dna::NodeGeometrySimulationOutput *sim,
    const bNode *from_node,
    const bNodeSocket *from_sock,
    int index);
void NOD_geometry_simulation_output_remove_item(blender::dna::NodeGeometrySimulationOutput *sim,
                                                blender::dna::NodeSimulationItem *item);
void NOD_geometry_simulation_output_clear_items(blender::dna::NodeGeometrySimulationOutput *sim);
void NOD_geometry_simulation_output_move_item(blender::dna::NodeGeometrySimulationOutput *sim,
                                              int from_index,
                                              int to_index);

/** \} */
