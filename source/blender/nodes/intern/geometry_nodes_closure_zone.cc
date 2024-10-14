/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_lazy_function.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_geometry_nodes_closure.hh"
#include "BKE_geometry_nodes_reference_set.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"
#include "BKE_node_tree_reference_lifetimes.hh"

#include "DEG_depsgraph_query.hh"

namespace blender::nodes {

using bke::node_tree_reference_lifetimes::ReferenceSetInfo;
using bke::node_tree_reference_lifetimes::ReferenceSetType;

class LazyFunctionForClosureZone : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bke::bNodeTreeZone &zone_;
  const bNode &output_bnode_;
  const ZoneBuildInfo &zone_info_;
  const ZoneBodyFunction &body_fn_;
  std::shared_ptr<bke::ClosureSignature> closure_signature_;

 public:
  LazyFunctionForClosureZone(const bNodeTree &btree,
                             const bke::bNodeTreeZone &zone,
                             ZoneBuildInfo &zone_info,
                             const ZoneBodyFunction &body_fn)
      : btree_(btree),
        zone_(zone),
        output_bnode_(*zone.output_node),
        zone_info_(zone_info),
        body_fn_(body_fn)
  {
    debug_name_ = "Closure Zone";

    initialize_zone_wrapper(zone, zone_info, body_fn, false, inputs_, outputs_);
    for (const auto item : body_fn.indices.inputs.reference_sets.items()) {
      const ReferenceSetInfo &reference_set =
          btree.runtime->reference_lifetimes_info->reference_sets[item.key];
      if (reference_set.type == ReferenceSetType::ClosureInputReferenceSet) {
        BLI_assert(&reference_set.socket->owner_node() != zone_.input_node);
      }
      if (reference_set.type == ReferenceSetType::ClosureOutputData) {
        if (&reference_set.socket->owner_node() == zone_.output_node) {
          /* This reference set comes from the caller of the closure and is not captured at the
           * place where the closure is created. */
          continue;
        }
      }
      zone_info.indices.inputs.reference_sets.add_new(
          item.key,
          inputs_.append_and_get_index_as("Reference Set",
                                          CPPType::get<bke::GeometryNodesReferenceSet>()));
    }

    /* All border links are used. */
    for (const int i : zone_.border_links.index_range()) {
      inputs_[zone_info.indices.inputs.border_links[i]].usage = lf::ValueUsage::Used;
    }

    const auto &storage = *static_cast<const NodeGeometryClosureOutput *>(output_bnode_.storage);

    std::shared_ptr<bke::SocketListSignature> input_list =
        std::make_shared<bke::SocketListSignature>();
    std::shared_ptr<bke::SocketListSignature> output_list =
        std::make_shared<bke::SocketListSignature>();

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const bNodeSocket &bsocket = zone_.input_node->output_socket(i);
      input_list->items.append({bsocket.typeinfo, bsocket.name});
    }
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const bNodeSocket &bsocket = zone_.output_node->input_socket(i);
      output_list->items.append({bsocket.typeinfo, bsocket.name});
    }
    closure_signature_ = std::make_shared<bke::ClosureSignature>(std::move(input_list),
                                                                 std::move(output_list));
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    for (const int i : zone_.border_links.index_range()) {
      params.set_output(zone_info_.indices.outputs.border_link_usages[i], true);
    }

    const auto &storage = *static_cast<const NodeGeometryClosureOutput *>(output_bnode_.storage);

    std::unique_ptr<ResourceScope> closure_scope = std::make_unique<ResourceScope>();
    LinearAllocator<> &closure_allocator = closure_scope->linear_allocator();

    lf::Graph &lf_graph = closure_scope->construct<lf::Graph>();
    lf::FunctionNode &lf_body_node = lf_graph.add_function(*body_fn_.function);
    bke::ClosureFunctionIndices closure_indices;

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeGeometryClosureInputItem &item = storage.input_items.items[i];
      const bNodeSocket &bsocket = zone_.input_node->output_socket(i);
      const CPPType &cpp_type = *bsocket.typeinfo->geometry_nodes_cpp_type;

      lf::GraphInputSocket &lf_graph_input = lf_graph.add_input(cpp_type, item.name);
      lf_graph.add_link(lf_graph_input, lf_body_node.input(body_fn_.indices.inputs.main[i]));

      lf::GraphOutputSocket &lf_graph_input_usage = lf_graph.add_output(
          CPPType::get<bool>(), "Usage: " + StringRef(item.name));
      lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.input_usages[i]),
                        lf_graph_input_usage);
    }
    closure_indices.inputs.main = lf_graph.graph_inputs().index_range().take_back(
        storage.input_items.items_num);
    closure_indices.outputs.input_usages = lf_graph.graph_outputs().index_range().take_back(
        storage.input_items.items_num);

    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeGeometryClosureOutputItem &item = storage.output_items.items[i];
      const bNodeSocket &bsocket = zone_.output_node->input_socket(i);
      const CPPType &cpp_type = *bsocket.typeinfo->geometry_nodes_cpp_type;

      lf::GraphOutputSocket &lf_graph_output = lf_graph.add_output(cpp_type, item.name);
      lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.main[i]), lf_graph_output);

      lf::GraphInputSocket &lf_graph_output_usage = lf_graph.add_input(
          CPPType::get<bool>(), "Usage: " + StringRef(item.name));
      lf_graph.add_link(lf_graph_output_usage,
                        lf_body_node.input(body_fn_.indices.inputs.output_usages[i]));
    }
    closure_indices.outputs.main = lf_graph.graph_outputs().index_range().take_back(
        storage.output_items.items_num);
    closure_indices.inputs.output_usages = lf_graph.graph_inputs().index_range().take_back(
        storage.output_items.items_num);

    for (const int i : zone_.border_links.index_range()) {
      const CPPType &cpp_type = *zone_.border_links[i]->tosock->typeinfo->geometry_nodes_cpp_type;
      void *input_ptr = params.try_get_input_data_ptr(zone_info_.indices.inputs.border_links[i]);
      void *stored_ptr = closure_allocator.allocate(cpp_type.size(), cpp_type.alignment());
      cpp_type.move_construct(input_ptr, stored_ptr);
      if (!cpp_type.is_trivially_destructible()) {
        closure_scope->add_destruct_call(
            [&cpp_type, stored_ptr]() { cpp_type.destruct(stored_ptr); });
      }
      lf_body_node.input(body_fn_.indices.inputs.border_links[i]).set_default_value(stored_ptr);
    }

    for (const auto &item : body_fn_.indices.inputs.reference_sets.items()) {
      const ReferenceSetInfo &reference_set =
          btree_.runtime->reference_lifetimes_info->reference_sets[item.key];
      if (reference_set.type == ReferenceSetType::ClosureOutputData) {
        const bNodeSocket &socket = *reference_set.socket;
        const bNode &node = socket.owner_node();
        if (&node == zone_.output_node) {
          /* This reference set is passed in by the code that invokes the closure. */
          lf::GraphInputSocket &lf_graph_input = lf_graph.add_input(
              CPPType::get<bke::GeometryNodesReferenceSet>(),
              StringRef("Reference Set: ") + reference_set.socket->name);
          lf_graph.add_link(
              lf_graph_input,
              lf_body_node.input(body_fn_.indices.inputs.reference_sets.lookup(item.key)));
          closure_indices.inputs.output_data_reference_sets.add_new(reference_set.socket->index(),
                                                                    lf_graph_input.index());
          continue;
        }
      }

      auto &input_reference_set = *params.try_get_input_data_ptr<bke::GeometryNodesReferenceSet>(
          zone_info_.indices.inputs.reference_sets.lookup(item.key));
      auto &stored = closure_scope->construct<bke::GeometryNodesReferenceSet>(
          std::move(input_reference_set));
      lf_body_node.input(body_fn_.indices.inputs.reference_sets.lookup(item.key))
          .set_default_value(&stored);
    }

    bNodeTree &btree_orig = *reinterpret_cast<bNodeTree *>(
        DEG_get_original_id(const_cast<ID *>(&btree_.id)));
    if (btree_orig.runtime->logged_zone_graphs) {
      std::lock_guard lock{btree_orig.runtime->logged_zone_graphs->mutex};
      btree_orig.runtime->logged_zone_graphs->graph_by_zone_id.lookup_or_add_cb(
          output_bnode_.identifier, [&]() { return lf_graph.to_dot(); });
    }

    lf_graph.update_node_indices();

    lf::GraphExecutor &lf_graph_executor = closure_scope->construct<lf::GraphExecutor>(
        lf_graph, nullptr, nullptr, nullptr);

    bke::ClosurePtr closure{MEM_new<bke::Closure>(__func__,
                                                  closure_signature_,
                                                  std::move(closure_scope),
                                                  lf_graph_executor,
                                                  closure_indices)};

    params.set_output(zone_info_.indices.outputs.main[0],
                      bke::SocketValueVariant(std::move(closure)));
  }
};

LazyFunction &build_closure_zone_lazy_function(ResourceScope &scope,
                                               const bNodeTree &btree,
                                               const bke::bNodeTreeZone &zone,
                                               ZoneBuildInfo &zone_info,
                                               const ZoneBodyFunction &body_fn)
{
  return scope.construct<LazyFunctionForClosureZone>(btree, zone, zone_info, body_fn);
}

}  // namespace blender::nodes
