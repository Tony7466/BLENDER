/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"
#include "NOD_zone_socket_items.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLI_string.h"

#include "BKE_bake_items_socket.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bake_cc {

NODE_STORAGE_FUNCS(NodeGeometryBake)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const NodeGeometryBake &storage = node_storage(*node);

  for (const int i : IndexRange(storage.items_num)) {
    const NodeGeometryBakeItem &item = storage.items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const StringRef name = item.name;
    const std::string identifier = BakeItemsAccessor::socket_identifier_for_item(item);
    auto &input_decl = b.add_input(socket_type, name, identifier);
    auto &output_decl = b.add_output(socket_type, name, identifier);
    if (socket_type_supports_fields(socket_type)) {
      input_decl.supports_field();
      output_decl.dependent_field({input_decl.input_index()});
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__");
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  UNUSED_VARS(layout, C, ptr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryBake *data = MEM_cnew<NodeGeometryBake>(__func__);

  data->items = MEM_cnew_array<NodeGeometryBakeItem>(1, __func__);
  data->items_num = 1;

  NodeGeometryBakeItem &item = data->items[0];
  item.name = BLI_strdup("Geometry");
  item.identifier = data->next_identifier++;
  item.attribute_domain = ATTR_DOMAIN_POINT;
  item.socket_type = SOCK_GEOMETRY;

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<BakeItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryBake &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeGeometryBake>(__func__, src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<BakeItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  return socket_items::try_add_item_via_any_extend_socket<BakeItemsAccessor>(
      *ntree, *node, *node, *link);
}

static const CPPType &get_item_cpp_type(const eNodeSocketDatatype socket_type)
{
  const char *socket_idname = nodeStaticSocketType(socket_type, 0);
  const bNodeSocketType *typeinfo = nodeSocketTypeFind(socket_idname);
  BLI_assert(typeinfo);
  BLI_assert(typeinfo->geometry_nodes_cpp_type);
  return *typeinfo->geometry_nodes_cpp_type;
}

static bke::bake::BakeSocketConfig make_bake_socket_config(
    const Span<NodeGeometryBakeItem> bake_items)
{
  bke::bake::BakeSocketConfig config;
  const int items_num = bake_items.size();
  config.domains.resize(items_num);
  config.types.resize(items_num);
  config.geometries_by_attribute.resize(items_num);

  int last_geometry_index = -1;
  for (const int item_i : bake_items.index_range()) {
    const NodeGeometryBakeItem &item = bake_items[item_i];
    config.types[item_i] = eNodeSocketDatatype(item.socket_type);
    config.domains[item_i] = eAttrDomain(item.attribute_domain);
    if (item.socket_type == SOCK_GEOMETRY) {
      last_geometry_index = item_i;
    }
    else if (last_geometry_index != -1) {
      config.geometries_by_attribute[item_i].append(last_geometry_index);
    }
  }
  return config;
}

class LazyFunctionForBakeNode final : public LazyFunction {
  const bNode &node_;
  Span<NodeGeometryBakeItem> bake_items_;
  bke::bake::BakeSocketConfig bake_socket_config_;

 public:
  LazyFunctionForBakeNode(const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Bake";
    const NodeGeometryBake &storage = node_storage(node);
    bake_items_ = {storage.items, storage.items_num};

    MutableSpan<int> lf_index_by_bsocket = lf_graph_info.mapping.lf_index_by_bsocket;

    for (const int i : bake_items_.index_range()) {
      const NodeGeometryBakeItem &item = bake_items_[i];
      const bNodeSocket &input_bsocket = node.input_socket(i);
      const bNodeSocket &output_bsocket = node.output_socket(i);
      const CPPType &type = get_item_cpp_type(eNodeSocketDatatype(item.socket_type));
      lf_index_by_bsocket[input_bsocket.index_in_tree()] = inputs_.append_and_get_index_as(
          item.name, type, lf::ValueUsage::Maybe);
      lf_index_by_bsocket[output_bsocket.index_in_tree()] = outputs_.append_and_get_index_as(
          item.name, type);
    }

    bake_socket_config_ = make_bake_socket_config(bake_items_);
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    GeoNodesLFUserData &user_data = *static_cast<GeoNodesLFUserData *>(context.user_data);
    if (!user_data.call_data->self_object()) {
      /* The self object is currently required for generating anonymous attribute names. */
      params.set_default_remaining_outputs();
      return;
    }
    if (!user_data.call_data->bake_params) {
      params.set_default_remaining_outputs();
      return;
    }
    std::optional<FoundNestedNodeID> found_id = find_nested_node_id(user_data, node_.identifier);
    if (!found_id) {
      params.set_default_remaining_outputs();
      return;
    }
    if (found_id->is_in_loop) {
      params.set_default_remaining_outputs();
      return;
    }
    BakeNodeBehavior *behavior = user_data.call_data->bake_params->get(found_id->id);
    if (!behavior) {
      params.set_default_remaining_outputs();
      return;
    }
    if (auto *info = std::get_if<sim_output::ReadSingle>(behavior)) {
      this->output_cached_state(params, user_data, info->state);
    }
    else if (auto *info = std::get_if<sim_output::ReadInterpolated>(behavior)) {
      this->output_mixed_cached_state(params,
                                      *user_data.call_data->self_object(),
                                      *user_data.compute_context,
                                      info->prev_state,
                                      info->next_state,
                                      info->mix_factor);
    }
    else if (std::get_if<sim_output::PassThrough>(behavior)) {
      this->pass_through(params, user_data);
    }
    else if (auto *info = std::get_if<sim_output::StoreNewState>(behavior)) {
      this->store(params, user_data, *info);
    }
    else {
      BLI_assert_unreachable();
    }
  }

  void pass_through(lf::Params &params, GeoNodesLFUserData &user_data) const
  {
    std::optional<bke::bake::BakeState> bake_state = this->get_bake_state_from_inputs(params);
    if (!bake_state) {
      /* Wait for inputs to be computed. */
      return;
    }
    Array<void *> output_values(bake_items_.size());
    for (const int i : bake_items_.index_range()) {
      output_values[i] = params.get_output_data_ptr(i);
    }
    this->move_bake_state_to_values(std::move(*bake_state),
                                    *user_data.call_data->self_object(),
                                    *user_data.compute_context,
                                    output_values);
    for (const int i : bake_items_.index_range()) {
      params.output_set(i);
    }
  }

  void store(lf::Params &params,
             GeoNodesLFUserData &user_data,
             const sim_output::StoreNewState &info) const
  {
    std::optional<bke::bake::BakeState> bake_state = this->get_bake_state_from_inputs(params);
    if (!bake_state) {
      /* Wait for inputs to be computed. */
      return;
    }
    this->output_cached_state(params, user_data, *bake_state);
    info.store_fn(std::move(*bake_state));
  }

  void output_cached_state(lf::Params &params,
                           GeoNodesLFUserData &user_data,
                           const bke::bake::BakeStateRef &bake_state) const
  {
    Array<void *> output_values(bake_items_.size());
    for (const int i : bake_items_.index_range()) {
      output_values[i] = params.get_output_data_ptr(i);
    }
    this->copy_bake_state_to_values(bake_state,
                                    *user_data.call_data->self_object(),
                                    *user_data.compute_context,
                                    output_values);
    for (const int i : bake_items_.index_range()) {
      params.output_set(i);
    }
  }

  void output_mixed_cached_state(lf::Params &params,
                                 const Object &self_object,
                                 const ComputeContext &compute_context,
                                 const bke::bake::BakeStateRef &prev_state,
                                 const bke::bake::BakeStateRef &next_state,
                                 const float mix_factor) const
  {
    Array<void *> output_values(bake_items_.size());
    for (const int i : bake_items_.index_range()) {
      output_values[i] = params.get_output_data_ptr(i);
    }
    this->copy_bake_state_to_values(prev_state, self_object, compute_context, output_values);

    Array<void *> next_values(bake_items_.size());
    LinearAllocator<> allocator;
    for (const int i : bake_items_.index_range()) {
      const CPPType &type = *outputs_[i].type;
      next_values[i] = allocator.allocate(type.size(), type.alignment());
    }
    this->copy_bake_state_to_values(next_state, self_object, compute_context, next_values);

    for (const int i : bake_items_.index_range()) {
      mix_baked_data_item(eNodeSocketDatatype(bake_items_[i].socket_type),
                          output_values[i],
                          next_values[i],
                          mix_factor);
    }

    for (const int i : bake_items_.index_range()) {
      const CPPType &type = *outputs_[i].type;
      type.destruct(next_values[i]);
    }

    for (const int i : bake_items_.index_range()) {
      params.output_set(i);
    }
  }

  std::optional<bke::bake::BakeState> get_bake_state_from_inputs(lf::Params &params) const
  {
    Array<void *> input_values(bake_items_.size());
    for (const int i : bake_items_.index_range()) {
      input_values[i] = params.try_get_input_data_ptr_or_request(i);
    }
    if (input_values.as_span().contains(nullptr)) {
      /* Wait for inputs to be computed. */
      return std::nullopt;
    }

    Array<std::unique_ptr<bke::bake::BakeItem>> bake_items =
        bke::bake::move_socket_values_to_bake_items(input_values, bake_socket_config_);

    bke::bake::BakeState bake_state;
    for (const int i : bake_items_.index_range()) {
      const NodeGeometryBakeItem &item = bake_items_[i];
      std::unique_ptr<bke::bake::BakeItem> &bake_item = bake_items[i];
      if (bake_item) {
        bake_state.items_by_id.add_new(item.identifier, std::move(bake_item));
      }
    }
    return bake_state;
  }

  void move_bake_state_to_values(bke::bake::BakeState bake_state,
                                 const Object &self_object,
                                 const ComputeContext &compute_context,
                                 Span<void *> r_output_values) const
  {
    Vector<bke::bake::BakeItem *> bake_items;
    for (const NodeGeometryBakeItem &item : bake_items_) {
      std::unique_ptr<bke::bake::BakeItem> *bake_item = bake_state.items_by_id.lookup_ptr(
          item.identifier);
      bake_items.append(bake_item ? bake_item->get() : nullptr);
    }
    bke::bake::move_bake_items_to_socket_values(
        bake_items,
        bake_socket_config_,
        [&](const int i, const CPPType &type) {
          return this->make_attribute_field(self_object, compute_context, bake_items_[i], type);
        },
        r_output_values);
  }

  void copy_bake_state_to_values(const bke::bake::BakeStateRef &bake_state,
                                 const Object &self_object,
                                 const ComputeContext &compute_context,
                                 Span<void *> r_output_values) const
  {
    Vector<const bke::bake::BakeItem *> bake_items;
    for (const NodeGeometryBakeItem &item : bake_items_) {
      const bke::bake::BakeItem *const *bake_item = bake_state.items_by_id.lookup_ptr(
          item.identifier);
      bake_items.append(bake_item ? *bake_item : nullptr);
    }
    bke::bake::copy_bake_items_to_socket_values(
        bake_items,
        bake_socket_config_,
        [&](const int i, const CPPType &type) {
          return this->make_attribute_field(self_object, compute_context, bake_items_[i], type);
        },
        r_output_values);
  }

  std::shared_ptr<AnonymousAttributeFieldInput> make_attribute_field(
      const Object &self_object,
      const ComputeContext &compute_context,
      const NodeGeometryBakeItem &item,
      const CPPType &type) const
  {
    AnonymousAttributeIDPtr attribute_id = AnonymousAttributeIDPtr(
        MEM_new<NodeAnonymousAttributeID>(__func__,
                                          self_object,
                                          compute_context,
                                          node_,
                                          std::to_string(item.identifier),
                                          item.name));
    return std::make_shared<AnonymousAttributeFieldInput>(
        attribute_id, type, node_.label_or_name());
  }
};

static void node_rna(StructRNA *srna)
{
  UNUSED_VARS(srna);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.insert_link = node_insert_link;
  node_type_storage(&ntype, "NodeGeometryBake", node_free_storage, node_copy_storage);
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bake_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_bake_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNode>(node, lf_graph_info);
}

};  // namespace blender::nodes
