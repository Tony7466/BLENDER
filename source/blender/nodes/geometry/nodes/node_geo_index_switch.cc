/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_array_utils.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "FN_field_cpp_type.hh"

namespace blender::nodes::node_geo_index_switch_cc {

NODE_STORAGE_FUNCS(NodeIndexSwitch)

static void node_declare(NodeDeclarationBuilder &b)
{
  // TODO: Only supports field if data type supports it.
  b.add_input<decl::Int>("Index").supports_field();
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const NodeIndexSwitch &storage = node_storage(*node);
  const eNodeSocketDatatype data_type = eNodeSocketDatatype(storage.data_type);
  const bool supports_fields = socket_type_supports_fields(data_type);

  const Span<IndexSwitchItem> items = storage.items_span();

  for (const int i : items.index_range()) {
    const std::string identifier = SimulationItemsAccessor::socket_identifier_for_item(item);
    auto &input = b.add_input(data_type, std::to_string(i), std::move(identifier));
    if (supports_fields) {
      input.supports_field();
    }
  }

  auto &output = b.add_output(data_type, "Output");
  if (supports_fields) {
    output.dependent_field().reference_pass_all();
  }
  else if (data_type == SOCK_GEOMETRY) {
    output.propagate_all();
  }

  b.add_input<decl::Extend>("", "__extend__");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeIndexSwitch *data = MEM_cnew<NodeIndexSwitch>(__func__);
  data->data_type = SOCK_GEOMETRY;
  data->next_identifier = 0;

  data->items = MEM_cnew_array<IndexSwitchItem>(1, __func__);
  data->items[0].identifier = data->next_identifier++;
  data->items_num = 1;

  node->storage = data;
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (params.in_out() == SOCK_OUT) {
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeIndexSwitch");
      node_storage(node).data_type = params.socket.type;
      params.update_and_connect_available_socket(node, "Output");
    });
  }
  else {
    if (params.node_tree().typeinfo->validate_link(eNodeSocketDatatype(params.other_socket().type),
                                                   SOCK_INT))
    {
      params.add_item(IFACE_("Index"), [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("GeometryNodeIndexSwitch");
        params.update_and_connect_available_socket(node, "Index");
      });
    }
  }
}

class IndexSwitchFunction : public mf::MultiFunction {
  mf::Signature signature_;
  Array<std::string> debug_names_;

 public:
  IndexSwitchFunction(const CPPType &type, const int items_num)
  {
    mf::SignatureBuilder builder{"Index Switch", signature_};
    builder.single_input<int>("Index");
    debug_names_.reinitialize(items_num);
    for (const int i : IndexRange(items_num)) {
      debug_names_[i] = std::to_string(i);
      builder.single_input(debug_names_[i].c_str(), type);
    }
    builder.single_output("Output", type);
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const
  {
    const VArray<int> indices = params.readonly_single_input<int>(0, "Index");

    IndexMaskMemory memory;
    Array<IndexMask> masks(signature_.params.size() - 2);
    // TODO: Invalid indices.
    IndexMask::from_groups<int64_t>(
        mask, memory, [&](const int64_t i) { return indices[i]; }, masks);

    int64_t all_valid_indices_size = 0;
    for (const IndexMask &index_mask : masks) {
      all_valid_indices_size += index_mask.size();
    }

    GMutableSpan outputs = params.uninitialized_single_output(signature_.params.size() - 1,
                                                              "Output");
    const CPPType &type = outputs.type();

    if (all_valid_indices_size != mask.size()) {
      type.fill_construct_n(type.default_value(), outputs.data(), outputs.size());
    }

    for (const int i : masks.index_range()) {
      if (masks[i].is_empty()) {
        continue;
      }
      const GVArray inputs = params.readonly_single_input(i + 1);
      array_utils::copy(inputs, masks[i], outputs);
    }
  }
};

class LazyFunctionForIndexSwitchNode : public LazyFunction {
 private:
  bool can_be_field_ = false;

 public:
  LazyFunctionForIndexSwitchNode(const bNode &node)
  {
    const NodeIndexSwitch &storage = node_storage(node);
    const eNodeSocketDatatype data_type = eNodeSocketDatatype(storage.data_type);
    can_be_field_ = socket_type_supports_fields(data_type);

    const CPPType &cpp_type = *node.output_socket(0).typeinfo->geometry_nodes_cpp_type;

    debug_name_ = node.name;
    inputs_.append_as("Index", CPPType::get<ValueOrField<int>>());
    for (const int i : storage.items_span().index_range().drop_front(1)) {
      inputs_.append_as(node.input_socket(i).identifier, cpp_type, lf::ValueUsage::Maybe);
    }
    outputs_.append_as("Value", cpp_type);
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const ValueOrField<int> index = params.get_input<ValueOrField<int>>(0);
    if (index.is_field() && can_be_field_) {
      Field<int> index_field = index.as_field();
      if (index_field.node().depends_on_input()) {
        this->execute_field(index.as_field(), params);
        return;
      }
      const bool index_int = fn::evaluate_constant_field(index_field);
      this->execute_single(index_int, params);
      return;
    }
    this->execute_single(index.as_value(), params);
  }

  void execute_single(const int index, lf::Params &params) const
  {
    constexpr int values_index_offset = 1;
    for (const int i : inputs_.index_range().drop_front(1)) {
      if (i != index) {
        params.set_input_unused(i + values_index_offset);
      }
    }
    void *value_to_forward = params.try_get_input_data_ptr_or_request(index + values_index_offset);
    if (value_to_forward == nullptr) {
      /* Try again when the value is available. */
      return;
    }

    const CPPType &type = *outputs_[0].type;
    void *output_ptr = params.get_output_data_ptr(0);
    type.move_construct(value_to_forward, output_ptr);
    params.output_set(0);
  }

  void execute_field(Field<int> index, lf::Params &params) const
  {
    Array<void *, 8> input_values(inputs_.size());
    for (const int i : inputs_.index_range()) {
      input_values[i] = params.try_get_input_data_ptr_or_request(i);
    }
    if (input_values.as_span().contains(nullptr)) {
      /* Try again when inputs are available. */
      return;
    }

    const CPPType &type = *outputs_[0].type;
    const fn::ValueOrFieldCPPType &value_or_field_type = *fn::ValueOrFieldCPPType::get_from_self(
        type);
    const CPPType &value_type = value_or_field_type.value;

    Vector<GField> input_fields({std::move(index)});
    for (const int i : inputs_.index_range().drop_front(1)) {
      input_fields.append(value_or_field_type.as_field(input_values[i]));
    }

    std::unique_ptr<mf::MultiFunction> switch_fn = std::make_unique<IndexSwitchFunction>(
        value_type, inputs_.size() - 1);
    GField output_field(FieldOperation::Create(std::move(switch_fn), std::move(input_fields)));

    void *output_ptr = params.get_output_data_ptr(0);
    value_or_field_type.construct_from_field(output_ptr, std::move(output_field));
    params.output_set(0);
  }
};

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(
      srna,
      "data_type",
      "Data Type",
      "",
      rna_enum_node_socket_data_type_items,
      NOD_storage_enum_accessors(data_type),
      SOCK_GEOMETRY,
      [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        return enum_items_filter(rna_enum_node_socket_data_type_items,
                                 [](const EnumPropertyItem &item) -> bool {
                                   return ELEM(item.value,
                                               SOCK_FLOAT,
                                               SOCK_INT,
                                               SOCK_BOOLEAN,
                                               SOCK_ROTATION,
                                               SOCK_VECTOR,
                                               SOCK_STRING,
                                               SOCK_RGBA,
                                               SOCK_GEOMETRY,
                                               SOCK_OBJECT,
                                               SOCK_COLLECTION,
                                               SOCK_TEXTURE,
                                               SOCK_MATERIAL,
                                               SOCK_IMAGE);
                                 });
      });
}

static void register_node()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SWITCH, "Switch", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  node_type_storage(
      &ntype, "NodeIndexSwitch", node_free_standard_storage, node_copy_standard_storage);
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(register_node)

}  // namespace blender::nodes::node_geo_index_switch_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_index_switch_node_lazy_function(const bNode &node)
{
  using namespace node_geo_index_switch_cc;
  BLI_assert(node.type == GEO_NODE_SWITCH);
  return std::make_unique<LazyFunctionForIndexSwitchNode>(node);
}

}  // namespace blender::nodes

blender::Span<IndexSwitchItem> NodeIndexSwitch::items_span() const
{
  return blender::Span<IndexSwitchItem>(items, items_num);
}

blender::MutableSpan<IndexSwitchItem> NodeIndexSwitch::items_span()
{
  return blender::MutableSpan<IndexSwitchItem>(items, items_num);
}
