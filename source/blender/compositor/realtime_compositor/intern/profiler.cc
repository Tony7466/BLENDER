/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_timeit.hh"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "NOD_derived_node_tree.hh"

#include "COM_profiler.hh"

namespace blender::realtime_compositor {

void Profiler::reset()
{
  nodes_evaluation_times_.clear();
}

void Profiler::set_node_evaluation_time(nodes::DNode node, timeit::Nanoseconds time)
{
  nodes_evaluation_times_.add_or_modify(
      node.instance_key(),
      [&](timeit::Nanoseconds *total_time) { *total_time = time; },
      [&](timeit::Nanoseconds *total_time) { *total_time += time; });
}

timeit::Nanoseconds Profiler::accumulate_node_group_times(const nodes::DTreeContext &tree_context)
{
  timeit::Nanoseconds tree_evaluation_time = timeit::Nanoseconds::zero();

  for (const bNode *b_node : tree_context.btree().all_nodes()) {
    const nodes::DNode node = nodes::DNode(&tree_context, b_node);

    if (node->type != NODE_GROUP) {
      /* Non-group node, no need to recurse into. Simply accumulate the node's evaluation time to
       * the current tree's evaluation time. Note that not everey node might have an evaluation
       * time stored, so default to zero. See the documentation on nodes_evaluation_times_ for more
       * information. */
      tree_evaluation_time += nodes_evaluation_times_.lookup_default(node.instance_key(),
                                                                     timeit::Nanoseconds::zero());
      continue;
    }

    if (node->id == nullptr) {
      /* Node group has lost link to its node tree. For example, due to missing linked file. */
      continue;
    }

    const timeit::Nanoseconds group_execution_time = this->accumulate_node_group_times(
        *tree_context.child_context(*b_node));

    /* Set evaluation time of the group node. */
    this->set_node_evaluation_time(node, group_execution_time);

    /* Add group evaluation time to the overall tree execution time. */
    tree_evaluation_time += group_execution_time;
  }

  return tree_evaluation_time;
}

void Profiler::finalize(const Scene &scene, const nodes::DerivedNodeTree &node_tree)
{
  /* Compute the evaluation time of all node groups starting from the root tree. */
  this->accumulate_node_group_times(node_tree.root_context());

  scene.runtime->compositor.per_node_execution_time = nodes_evaluation_times_;
}

}  // namespace blender::realtime_compositor
