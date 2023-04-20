/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket.h"

#include "node_geometry_util.hh"

namespace blender::nodes {

template<typename T> static void copy_typed_initial_simulation_state(lf::Params &params, int index)
{
  T *data = params.try_get_input_data_ptr_or_request<T>(index);
  if (data != nullptr) {
    /* First output parameter is "Delta Time", state item parameters start at index 1. */
    params.set_output(index + 1, std::move(*data));
  }
}

static void copy_initial_simulation_state(lf::Params &params, int index, short socket_type)
{
  switch (socket_type) {
    case SOCK_FLOAT:
      copy_typed_initial_simulation_state<ValueOrField<float>>(params, index);
      break;
    case SOCK_VECTOR:
      copy_typed_initial_simulation_state<ValueOrField<float3>>(params, index);
      break;
    case SOCK_RGBA:
      copy_typed_initial_simulation_state<ValueOrField<ColorGeometry4f>>(params, index);
      break;
    case SOCK_BOOLEAN:
      copy_typed_initial_simulation_state<ValueOrField<bool>>(params, index);
      break;
    case SOCK_INT:
      copy_typed_initial_simulation_state<ValueOrField<int>>(params, index);
      break;
    case SOCK_STRING:
      copy_typed_initial_simulation_state<ValueOrField<std::string>>(params, index);
      break;
    case SOCK_OBJECT:
      copy_typed_initial_simulation_state<Object *>(params, index);
      break;
    case SOCK_GEOMETRY:
      copy_typed_initial_simulation_state<GeometrySet>(params, index);
      break;
    case SOCK_COLLECTION:
      copy_typed_initial_simulation_state<Collection *>(params, index);
      break;
    case SOCK_TEXTURE:
      copy_typed_initial_simulation_state<Tex *>(params, index);
      break;
    case SOCK_IMAGE:
      copy_typed_initial_simulation_state<Image *>(params, index);
      break;
    case SOCK_MATERIAL:
      copy_typed_initial_simulation_state<Material *>(params, index);
      break;
    default:
      BLI_assert_unreachable();
      copy_typed_initial_simulation_state<GeometrySet>(params, index);
      break;
  }
}

template<typename T>
static void copy_typed_next_simulation_state(lf::Params &params,
                                             int index,
                                             const bke::sim::SimulationStateItem &state_item)
{
  if (auto *typed_state_item = dynamic_cast<const bke::sim::TypedSimulationStateItem<T> *>(
          &state_item)) {
    params.set_output(index + 1, typed_state_item->data());
  }
}

static void copy_next_simulation_state(lf::Params &params,
                                       int index,
                                       short socket_type,
                                       const bke::sim::SimulationStateItem &state_item)
{
  switch (socket_type) {
    case SOCK_FLOAT:
      copy_typed_next_simulation_state<ValueOrField<float>>(params, index, state_item);
      break;
    case SOCK_VECTOR:
      copy_typed_next_simulation_state<ValueOrField<float3>>(params, index, state_item);
      break;
    case SOCK_RGBA:
      copy_typed_next_simulation_state<ValueOrField<ColorGeometry4f>>(params, index, state_item);
      break;
    case SOCK_BOOLEAN:
      copy_typed_next_simulation_state<ValueOrField<bool>>(params, index, state_item);
      break;
    case SOCK_INT:
      copy_typed_next_simulation_state<ValueOrField<int>>(params, index, state_item);
      break;
    case SOCK_STRING:
      copy_typed_next_simulation_state<ValueOrField<std::string>>(params, index, state_item);
      break;
    case SOCK_OBJECT:
      copy_typed_next_simulation_state<Object *>(params, index, state_item);
      break;
    case SOCK_GEOMETRY:
      copy_typed_next_simulation_state<GeometrySet>(params, index, state_item);
      break;
    case SOCK_COLLECTION:
      copy_typed_next_simulation_state<Collection *>(params, index, state_item);
      break;
    case SOCK_TEXTURE:
      copy_typed_next_simulation_state<Tex *>(params, index, state_item);
      break;
    case SOCK_IMAGE:
      copy_typed_next_simulation_state<Image *>(params, index, state_item);
      break;
    case SOCK_MATERIAL:
      copy_typed_next_simulation_state<Material *>(params, index, state_item);
      break;
    default:
      BLI_assert_unreachable();
      copy_typed_next_simulation_state<GeometrySet>(params, index, state_item);
      break;
  }
}

}  // namespace blender::nodes

namespace blender::nodes::node_geo_simulation_input_cc {

NODE_STORAGE_FUNCS(NodeGeometrySimulationInput);

class LazyFunctionForSimulationInputNode final : public LazyFunction {
  int32_t output_node_id_;
  Span<NodeSimulationItem> simulation_items_;

 public:
  LazyFunctionForSimulationInputNode(const bNodeTree &node_tree, const bNode &node)
  {
    output_node_id_ = node_storage(node).output_node_id;
    const bNode &output_node = *node_tree.node_by_id(output_node_id_);
    const NodeGeometrySimulationOutput &storage = *static_cast<NodeGeometrySimulationOutput *>(
        output_node.storage);
    simulation_items_ = {storage.items, storage.items_num};
    outputs_.append_as("Delta Time", CPPType::get<ValueOrField<float>>());
    for (const NodeSimulationItem &item : Span(storage.items, storage.items_num)) {
      const CPPType &type = get_simulation_item_cpp_type(item);
      inputs_.append_as(item.name, type, lf::ValueUsage::Maybe);
      outputs_.append_as(item.name, type);
    }
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    GeoNodesLFUserData &user_data = *static_cast<GeoNodesLFUserData *>(context.user_data);
    GeoNodesModifierData &modifier_data = *user_data.modifier_data;

    if (modifier_data.current_simulation_state == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    if (!params.output_was_set(0)) {
      const float delta_time = modifier_data.simulation_time_delta;
      params.set_output(0, fn::ValueOrField<float>(delta_time));
    }

    const bke::sim::SimulationZoneID zone_id = get_simulation_zone_id(*user_data.compute_context,
                                                                      output_node_id_);

    const bke::sim::SimulationZoneState *prev_zone_state =
        modifier_data.prev_simulation_state == nullptr ?
            nullptr :
            modifier_data.prev_simulation_state->get_zone_state(zone_id);
    if (prev_zone_state == nullptr) {
      for (const int i : simulation_items_.index_range()) {
        if (params.output_was_set(i + 1)) {
          continue;
        }
        copy_initial_simulation_state(params, i, simulation_items_[i].socket_type);
      }
    }
    else {
      for (const int i : simulation_items_.index_range()) {
        if (i >= prev_zone_state->items.size()) {
          params.set_output(i + 1, GeometrySet());
          continue;
        }
        const bke::sim::SimulationStateItem *state_item = prev_zone_state->items[i].get();
        if (state_item != nullptr) {
          copy_next_simulation_state(params, i, simulation_items_[i].socket_type, *state_item);
        }
      }
    }
  }
};

}  // namespace blender::nodes::node_geo_simulation_input_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_simulation_input_lazy_function(const bNodeTree &node_tree,
                                                                 const bNode &node)
{
  namespace file_ns = blender::nodes::node_geo_simulation_input_cc;
  BLI_assert(node.type == GEO_NODE_SIMULATION_INPUT);
  return std::make_unique<file_ns::LazyFunctionForSimulationInputNode>(node_tree, node);
}

}  // namespace blender::nodes

namespace blender::nodes::node_geo_simulation_input_cc {

static void node_declare_dynamic(const bNodeTree &node_tree,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  const bNode *output_node = node_tree.node_by_id(node_storage(node).output_node_id);
  if (!output_node) {
    return;
  }

  std::unique_ptr<decl::Float> delta_time = std::make_unique<decl::Float>();
  delta_time->identifier = N_("Delta Time");
  delta_time->name = delta_time->identifier;
  delta_time->in_out = SOCK_OUT;
  r_declaration.outputs.append(std::move(delta_time));

  const NodeGeometrySimulationOutput &storage = *static_cast<const NodeGeometrySimulationOutput *>(
      output_node->storage);
  socket_declarations_for_simulation_items({storage.items, storage.items_num}, r_declaration);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySimulationInput *data = MEM_cnew<NodeGeometrySimulationInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  const bNode *output_node = ntree->node_by_id(node_storage(*node).output_node_id);
  if (!output_node) {
    return true;
  }

  NodeGeometrySimulationOutput &storage = *static_cast<NodeGeometrySimulationOutput *>(
      output_node->storage);

  if (link->tonode == node) {
    if (link->tosock->identifier == StringRef("__extend__")) {
      if (const NodeSimulationItem *item = simulation_item_add_from_socket(storage,
                                                                           *link->fromsock)) {
        update_node_declaration_and_sockets(*ntree, *node);
        link->tosock = nodeFindSocket(node, SOCK_IN, item->name);
      }
      else {
        return false;
      }
    }
  }
  else {
    BLI_assert(link->fromnode == node);
    if (link->fromsock->identifier == StringRef("__extend__")) {
      if (const NodeSimulationItem *item = simulation_item_add_from_socket(storage,
                                                                           *link->tosock)) {
        update_node_declaration_and_sockets(*ntree, *node);
        link->fromsock = nodeFindSocket(node, SOCK_OUT, item->name);
      }
      else {
        return false;
      }
    }
  }
  return true;
}

}  // namespace blender::nodes::node_geo_simulation_input_cc

void register_node_type_geo_simulation_input()
{
  namespace file_ns = blender::nodes::node_geo_simulation_input_cc;

  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_SIMULATION_INPUT, "Simulation Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = file_ns::node_init;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.insert_link = file_ns::node_insert_link;
  node_type_storage(&ntype,
                    "NodeGeometrySimulationInput",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  nodeRegisterType(&ntype);
}

bool NOD_geometry_simulation_input_pair_with_output(const bNodeTree *node_tree,
                                                    bNode *sim_input_node,
                                                    const bNode *sim_output_node)
{
  namespace file_ns = blender::nodes::node_geo_simulation_input_cc;

  BLI_assert(sim_input_node->type == GEO_NODE_SIMULATION_INPUT);
  if (sim_output_node->type != GEO_NODE_SIMULATION_OUTPUT) {
    return false;
  }

  /* Allow only one input paired to an output. */
  for (const bNode *other_input_node : node_tree->nodes_by_type("GeometryNodeSimulationInput")) {
    if (other_input_node != sim_input_node) {
      const NodeGeometrySimulationInput &other_storage = file_ns::node_storage(*other_input_node);
      if (other_storage.output_node_id == sim_output_node->identifier) {
        return false;
      }
    }
  }

  NodeGeometrySimulationInput &storage = file_ns::node_storage(*sim_input_node);
  storage.output_node_id = sim_output_node->identifier;
  return true;
}
