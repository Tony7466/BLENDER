/* SPDX-FileCopyrightText: 2013 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup depsgraph
 *
 * Core routines for how the Depsgraph works.
 */

#pragma once

namespace blender::deg {

struct Depsgraph;

/**
 * Flush updates from tagged nodes outwards until all affected nodes are tagged.
 * The is_visibility_update argument should be true iff the updates are part of the initial
 * evaluation when data-blocks becomes visible.
 */
void deg_graph_flush_updates(struct Depsgraph *graph, bool is_visibility_update);

/**
 * Clear tags from all operation nodes.
 */
void deg_graph_clear_tags(struct Depsgraph *graph);

}  // namespace blender::deg
