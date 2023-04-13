/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utils.h"

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_common.h"
#include "NOD_socket.h"

#include "node_geometry_util.hh"

namespace blender::nodes {

void socket_declarations_for_simulation_items(const Span<NodeSimulationItem> items,
                                              NodeDeclaration &r_declaration)
{
  for (const NodeSimulationItem &item : items) {
    switch (eNodeSocketDatatype(item.socket_type)) {
      case SOCK_GEOMETRY: {
        {
          std::unique_ptr<decl::Geometry> decl = std::make_unique<decl::Geometry>();
          decl->name = item.name ? item.name : "";
          decl->identifier = item.name ? item.name : "";
          decl->in_out = SOCK_IN;
          r_declaration.inputs.append(std::move(decl));
        }
        {
          std::unique_ptr<decl::Geometry> decl = std::make_unique<decl::Geometry>();
          decl->name = item.name ? item.name : "";
          decl->identifier = item.name ? item.name : "";
          decl->in_out = SOCK_OUT;
          r_declaration.outputs.append(std::move(decl));
        }
        break;
      }
      default:
        BLI_assert_unreachable();
    }
  }
  r_declaration.inputs.append(decl::create_extend_declaration(SOCK_IN));
  r_declaration.outputs.append(decl::create_extend_declaration(SOCK_OUT));
}

struct SimulationItemsUniqueNameArgs {
  NodeGeometrySimulationOutput *sim;
  const NodeSimulationItem *item;
};

static bool simulation_items_unique_name_check(void *arg, const char *name)
{
  const SimulationItemsUniqueNameArgs &args = *static_cast<const SimulationItemsUniqueNameArgs *>(
      arg);
  for (const NodeSimulationItem &item : Span(args.sim->items, args.sim->items_num)) {
    if (&item != args.item) {
      if (STREQ(item.name, name)) {
        return true;
      }
    }
  }
  if (STREQ(name, "Delta Time")) {
    return true;
  }
  return false;
}

const CPPType &get_simulation_item_cpp_type(const NodeSimulationItem &item)
{
  switch (item.socket_type) {
    case SOCK_GEOMETRY:
      return CPPType::get<GeometrySet>();
    default:
      BLI_assert_unreachable();
      return CPPType::get<GeometrySet>();
  }
}

}  // namespace blender::nodes

namespace blender::nodes::node_geo_simulation_output_cc {

NODE_STORAGE_FUNCS(NodeGeometrySimulationOutput);

struct EvalData {
  bool is_first_evaluation = true;
};

class LazyFunctionForSimulationOutputNode final : public LazyFunction {
  int32_t node_id_;
  Span<NodeSimulationItem> simulation_items_;

 public:
  LazyFunctionForSimulationOutputNode(const bNode &node) : node_id_(node.identifier)
  {
    const NodeGeometrySimulationOutput &storage = node_storage(node);
    simulation_items_ = {storage.items, storage.items_num};
    for (const NodeSimulationItem &item : Span(storage.items, storage.items_num)) {
      const CPPType &type = get_simulation_item_cpp_type(item);
      inputs_.append_as(item.name, type, lf::ValueUsage::Maybe);
      outputs_.append_as(item.name, type);
    }
  }

  void *init_storage(LinearAllocator<> &allocator) const
  {
    return allocator.construct<EvalData>().get();
  }

  void destruct_storage(void *storage) const
  {
    std::destroy_at(static_cast<EvalData *>(storage));
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    GeoNodesLFUserData &user_data = *static_cast<GeoNodesLFUserData *>(context.user_data);
    GeoNodesModifierData &modifier_data = *user_data.modifier_data;
    EvalData &eval_data = *static_cast<EvalData *>(context.storage);
    BLI_SCOPED_DEFER([&]() { eval_data.is_first_evaluation = false; });

    if (modifier_data.current_simulation_state == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    const bke::sim::SimulationZoneID zone_id = get_simulation_zone_id(*user_data.compute_context,
                                                                      node_id_);

    const bke::sim::SimulationZoneState *cached_zone_state =
        modifier_data.current_simulation_state->get_zone_state(zone_id);
    if (cached_zone_state != nullptr && eval_data.is_first_evaluation) {
      this->output_cached_state(params, *cached_zone_state);
      return;
    }

    if (modifier_data.current_simulation_state_for_write == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    bke::sim::SimulationZoneState &new_zone_state =
        modifier_data.current_simulation_state_for_write->get_zone_state_for_write(zone_id);
    new_zone_state.items.reinitialize(simulation_items_.size());

    bool all_available = true;
    for (const int i : simulation_items_.index_range()) {
      GeometrySet *input_geometry = params.try_get_input_data_ptr_or_request<GeometrySet>(i);
      if (input_geometry == nullptr) {
        all_available = false;
        continue;
      }
      input_geometry->ensure_owns_direct_data();
      new_zone_state.items[i] = std::make_unique<bke::sim::GeometrySimulationStateItem>(
          std::move(*input_geometry));
    }

    if (all_available) {
      this->output_cached_state(params, new_zone_state);
    }
  }

  void output_cached_state(lf::Params &params, const bke::sim::SimulationZoneState &state) const
  {
    for (const int i : simulation_items_.index_range()) {
      if (i >= state.items.size()) {
        continue;
      }
      const bke::sim::SimulationStateItem *item = state.items[i].get();
      if (item == nullptr) {
        continue;
      }
      if (auto *geometry_item = dynamic_cast<const bke::sim::GeometrySimulationStateItem *>(
              item)) {
        params.set_output(i, geometry_item->geometry());
      }
    }
    params.set_default_remaining_outputs();
  }
};

}  // namespace blender::nodes::node_geo_simulation_output_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_simulation_output_lazy_function(const bNode &node)
{
  namespace file_ns = blender::nodes::node_geo_simulation_output_cc;
  BLI_assert(node.type == GEO_NODE_SIMULATION_OUTPUT);
  return std::make_unique<file_ns::LazyFunctionForSimulationOutputNode>(node);
}

bke::sim::SimulationZoneID get_simulation_zone_id(const ComputeContext &compute_context,
                                                  const int output_node_id)
{
  bke::sim::SimulationZoneID zone_id;
  for (const ComputeContext *context = &compute_context; context != nullptr;
       context = context->parent()) {
    if (const auto *node_context = dynamic_cast<const bke::NodeGroupComputeContext *>(context)) {
      zone_id.node_ids.append(node_context->node_id());
    }
  }
  std::reverse(zone_id.node_ids.begin(), zone_id.node_ids.end());
  zone_id.node_ids.append(output_node_id);
  return zone_id;
}

}  // namespace blender::nodes

namespace blender::nodes::node_geo_simulation_output_cc {

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  const NodeGeometrySimulationOutput &storage = node_storage(node);
  socket_declarations_for_simulation_items({storage.items, storage.items_num}, r_declaration);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySimulationOutput *data = MEM_cnew<NodeGeometrySimulationOutput>(__func__);
  data->items = MEM_cnew_array<NodeSimulationItem>(1, __func__);
  data->items[0].name = BLI_strdup(DATA_("Geometry"));
  data->items[0].socket_type = SOCK_GEOMETRY;
  data->items_num = 1;
  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  if (!node->storage) {
    return;
  }
  NodeGeometrySimulationOutput &storage = node_storage(*node);
  for (NodeSimulationItem &item : MutableSpan(storage.items, storage.items_num)) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(storage.items);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometrySimulationOutput &src_storage = node_storage(*src_node);
  NodeGeometrySimulationOutput *dst_storage = MEM_cnew<NodeGeometrySimulationOutput>(__func__);

  dst_storage->items = MEM_cnew_array<NodeSimulationItem>(src_storage.items_num, __func__);
  dst_storage->items_num = src_storage.items_num;
  for (const int i : IndexRange(src_storage.items_num)) {
    if (char *name = src_storage.items[i].name) {
      dst_storage->items[i].name = BLI_strdup(name);
      dst_storage->items[i].socket_type = src_storage.items[i].socket_type;
    }
  }

  dst_node->storage = dst_storage;
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  NodeGeometrySimulationOutput &storage = node_storage(*node);
  if (link->tonode == node) {
    if (link->tosock->identifier == StringRef("__extend__")) {
      if (const NodeSimulationItem *item = node_geo_simulation_output_add_item_from_socket(
              &storage, link->fromnode, link->fromsock)) {
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
      if (const NodeSimulationItem *item = node_geo_simulation_output_add_item_from_socket(
              &storage, link->fromnode, link->tosock)) {
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

}  // namespace blender::nodes::node_geo_simulation_output_cc

void register_node_type_geo_simulation_output()
{
  namespace file_ns = blender::nodes::node_geo_simulation_output_cc;

  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_SIMULATION_OUTPUT, "Simulation Output", NODE_CLASS_INTERFACE);
  ntype.initfunc = file_ns::node_init;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.insert_link = file_ns::node_insert_link;
  node_type_storage(&ntype,
                    "NodeGeometrySimulationOutput",
                    file_ns::node_free_storage,
                    file_ns::node_copy_storage);
  nodeRegisterType(&ntype);
}

bNode *node_geo_simulation_output_find_node_by_data(bNodeTree *ntree,
                                                    const NodeGeometrySimulationOutput *sim)
{
  for (bNode *node : ntree->nodes_by_type("GeometryNodeSimulationOutput")) {
    if (node->storage == sim) {
      return node;
    }
  }
  return nullptr;
}

bNode *node_geo_simulation_output_find_node_by_item(bNodeTree *ntree,
                                                    const NodeSimulationItem *item)
{
  for (bNode *node : ntree->nodes_by_type("GeometryNodeSimulationOutput")) {
    NodeGeometrySimulationOutput *sim = static_cast<NodeGeometrySimulationOutput *>(node->storage);
    if (node_geo_simulation_output_contains_item(sim, item)) {
      return node;
    }
  }
  return nullptr;
}

bool node_geo_simulation_output_item_set_unique_name(NodeGeometrySimulationOutput *sim,
                                                     NodeSimulationItem *item,
                                                     const char *name)
{
  const char *defname = nodeStaticSocketLabel(item->socket_type, 0);

  char unique_name[MAX_NAME + 4];
  BLI_strncpy(unique_name, name, sizeof(unique_name));

  blender::nodes::SimulationItemsUniqueNameArgs args{sim, item};
  const bool name_changed = BLI_uniquename_cb(blender::nodes::simulation_items_unique_name_check,
                                              &args,
                                              defname,
                                              '.',
                                              unique_name,
                                              ARRAY_SIZE(unique_name));
  item->name = BLI_strdup(unique_name);
  return name_changed;
}

bool node_geo_simulation_output_contains_item(NodeGeometrySimulationOutput *sim,
                                              const NodeSimulationItem *item)
{
  const int index = item - sim->items;
  return index >= 0 && index < sim->items_num;
}

NodeSimulationItem *node_geo_simulation_output_get_active_item(NodeGeometrySimulationOutput *sim)
{
  if (sim->active_index >= 0 && sim->active_index < sim->items_num) {
    return &sim->items[sim->active_index];
  }
  return nullptr;
}

void node_geo_simulation_output_set_active_item(NodeGeometrySimulationOutput *sim,
                                                NodeSimulationItem *item)
{
  const int index = item - sim->items;
  if (index >= 0 && index < sim->items_num) {
    sim->active_index = index;
  }
}

NodeSimulationItem *node_geo_simulation_output_find_item(NodeGeometrySimulationOutput *sim,
                                                         const char *name)
{
  for (const int i : blender::IndexRange(sim->items_num)) {
    if (STREQ(sim->items[i].name, name)) {
      return &sim->items[i];
    }
  }
  return nullptr;
}

NodeSimulationItem *node_geo_simulation_output_add_item(NodeGeometrySimulationOutput *sim,
                                                        const short socket_type,
                                                        const char *name)
{
  return node_geo_simulation_output_insert_item(sim, socket_type, name, sim->items_num);
}

NodeSimulationItem *node_geo_simulation_output_insert_item(NodeGeometrySimulationOutput *sim,
                                                           const short socket_type,
                                                           const char *name,
                                                           int index)
{
  NodeSimulationItem *old_items = sim->items;
  sim->items = MEM_cnew_array<NodeSimulationItem>(sim->items_num + 1, __func__);
  for (const int i : blender::IndexRange(0, index)) {
    sim->items[i].name = old_items[i].name;
    sim->items[i].socket_type = old_items[i].socket_type;
  }
  for (const int i : blender::IndexRange(index, sim->items_num - index)) {
    sim->items[i + 1].name = old_items[i].name;
    sim->items[i + 1].socket_type = old_items[i].socket_type;
  }

  NodeSimulationItem &added_item = sim->items[index];
  node_geo_simulation_output_item_set_unique_name(sim, &added_item, name);
  added_item.socket_type = socket_type;

  sim->items_num++;
  MEM_SAFE_FREE(old_items);

  return &added_item;
}

NodeSimulationItem *node_geo_simulation_output_add_item_from_socket(
    NodeGeometrySimulationOutput *sim, const bNode * /*from_node*/, const bNodeSocket *from_sock)
{
  if (from_sock->type != SOCK_GEOMETRY) {
    return nullptr;
  }
  return node_geo_simulation_output_insert_item(
      sim, from_sock->type, from_sock->name, sim->items_num);
}

NodeSimulationItem *node_geo_simulation_output_insert_item_from_socket(
    NodeGeometrySimulationOutput *sim,
    const bNode * /*from_node*/,
    const bNodeSocket *from_sock,
    int index)
{
  return node_geo_simulation_output_insert_item(
      sim, from_sock->type, from_sock->name, index);
}

void node_geo_simulation_output_remove_item(NodeGeometrySimulationOutput *sim,
                                            NodeSimulationItem *item)
{
  const int index = item - sim->items;
  if (index < 0 || index >= sim->items_num) {
    return;
  }

  NodeSimulationItem *old_items = sim->items;
  sim->items = MEM_cnew_array<NodeSimulationItem>(sim->items_num - 1, __func__);
  for (const int i : blender::IndexRange(0, index)) {
    sim->items[i].name = old_items[i].name;
    sim->items[i].socket_type = old_items[i].socket_type;
  }
  for (const int i : blender::IndexRange(index, sim->items_num - index).drop_front(1)) {
    sim->items[i - 1].name = old_items[i].name;
    sim->items[i - 1].socket_type = old_items[i].socket_type;
  }

  MEM_SAFE_FREE(old_items[index].name);

  sim->items_num--;
  MEM_SAFE_FREE(old_items);
}

void node_geo_simulation_output_clear_items(struct NodeGeometrySimulationOutput *sim)
{
  for (NodeSimulationItem &item : blender::MutableSpan(sim->items, sim->items_num)) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(sim->items);
  sim->items = nullptr;
  sim->items_num = 0;
}

void node_geo_simulation_output_move_item(NodeGeometrySimulationOutput *sim,
                                          int from_index,
                                          int to_index)
{
  BLI_assert(from_index >= 0 && from_index < sim->items_num);
  BLI_assert(to_index >= 0 && to_index < sim->items_num);

  if (from_index == to_index) {
    return;
  }
  else if (from_index < to_index) {
    const NodeSimulationItem tmp = sim->items[from_index];
    for (int i = from_index; i < to_index; ++i) {
      sim->items[i] = sim->items[i + 1];
    }
    sim->items[to_index] = tmp;
  }
  else /* from_index > to_index */ {
    const NodeSimulationItem tmp = sim->items[from_index];
    for (int i = from_index; i > to_index; --i) {
      sim->items[i] = sim->items[i - 1];
    }
    sim->items[to_index] = tmp;
  }
}
