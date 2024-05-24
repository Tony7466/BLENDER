/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_map.hh"
#include "BLI_timeit.hh"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BKE_scene_runtime.hh"

#include "NOD_derived_node_tree.hh"

namespace blender::realtime_compositor {

/* -------------------------------------------------------------------------------------------------
 * Profiler
 *
 * A class that profiles the evaluation of the compositor and tracks information like the
 * evaluation time of every node. */
class Profiler {
 private:
  /* Stores the evaluation time of each node instance keyed by its instance key. Note that
   * pixel-wise nodes like Math nodes will not be measured, that's because they are compiled
   * together with other pixel-wise operations in a single operation, so we can't measure the
   * evaluation time of each individual node. */
  Map<bNodeInstanceKey, timeit::Nanoseconds> nodes_evaluation_times_;

 public:
  /* Resets the profiler by clearing the nodes evaluation times. This should be called before every
   * evaluation. */
  void reset();

  /* Set the evaluation time of the given node. */
  void set_node_evaluation_time(nodes::DNode node, timeit::Nanoseconds time);

  /* Finalize profiling by computing node group times and setting the timing information to the
   * scene compositor runtime profile data. This should be called after every evaluation. */
  void finalize(const Scene &scene, const nodes::DerivedNodeTree &node_tree);

 private:
  /* Computes the evaluation time of every group node inside the given tree context recursively by
   * accumulating the evaluation time of its nodes, setting the computed time to the group nodes.
   * The time is returned since the method is called recursively. */
  timeit::Nanoseconds accumulate_node_group_times(const nodes::DTreeContext &tree_context);
};

}  // namespace blender::realtime_compositor
