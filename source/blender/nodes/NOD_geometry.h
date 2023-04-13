/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct bNodeTreeType *ntreeType_Geometry;

void register_node_type_geo_custom_group(bNodeType *ntype);

/* -------------------------------------------------------------------- */
/** \name Simulation Input Node
 * \{ */

struct bNode *node_geo_simulation_input_get_paired_output(
    struct bNodeTree *node_tree, const struct bNode *simulation_input_node);

/**
 * Pair a simulation input node with an output node.
 * @return True if pairing the node was successful.
 */
bool node_geo_simulation_input_pair_with_output(const struct bNodeTree *node_tree,
                                                struct bNode *simulation_input_node,
                                                const struct bNode *simulation_output_node);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Simulation Output Node
 * \{ */

/** Set a unique item name.
 * @return True if the unique name differs from the original name.
 */
bool node_geo_simulation_output_item_set_unique_name(struct NodeGeometrySimulationOutput *sim,
                                                     struct NodeSimulationItem *item,
                                                     const char *name);

/** Find the node owning this simulation output data. */
bNode *node_geo_simulation_output_find_node_by_data(
    struct bNodeTree *ntree, const struct NodeGeometrySimulationOutput *sim);
/** Find the node owning this simulation state item. */
bNode *node_geo_simulation_output_find_node_by_item(struct bNodeTree *ntree,
                                                    const struct NodeSimulationItem *item);

bool node_geo_simulation_output_contains_item(struct NodeGeometrySimulationOutput *sim,
                                              const struct NodeSimulationItem *item);
struct NodeSimulationItem *node_geo_simulation_output_get_active_item(
    struct NodeGeometrySimulationOutput *sim);
void node_geo_simulation_output_set_active_item(struct NodeGeometrySimulationOutput *sim,
                                                struct NodeSimulationItem *item);
struct NodeSimulationItem *node_geo_simulation_output_find_item(
    struct NodeGeometrySimulationOutput *sim, const char *name);
struct NodeSimulationItem *node_geo_simulation_output_add_item(
    struct NodeGeometrySimulationOutput *sim, short socket_type, const char *name);
struct NodeSimulationItem *node_geo_simulation_output_insert_item(
    struct NodeGeometrySimulationOutput *sim, short socket_type, const char *name, int index);
struct NodeSimulationItem *node_geo_simulation_output_add_item_from_socket(
    struct NodeGeometrySimulationOutput *sim,
    const struct bNode *from_node,
    const struct bNodeSocket *from_sock);
struct NodeSimulationItem *node_geo_simulation_output_insert_item_from_socket(
    struct NodeGeometrySimulationOutput *sim,
    const struct bNode *from_node,
    const struct bNodeSocket *from_sock,
    int index);
void node_geo_simulation_output_remove_item(struct NodeGeometrySimulationOutput *sim,
                                            struct NodeSimulationItem *item);
void node_geo_simulation_output_clear_items(struct NodeGeometrySimulationOutput *sim);
void node_geo_simulation_output_move_item(struct NodeGeometrySimulationOutput *sim,
                                          int from_index,
                                          int to_index);

/** \} */

#ifdef __cplusplus
}
#endif
