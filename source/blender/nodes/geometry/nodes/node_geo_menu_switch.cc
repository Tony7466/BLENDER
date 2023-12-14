/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "DNA_node_types.h"

#include "BLI_string.h"

#include "FN_multi_function.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "BKE_node_socket_value_cpp_type.hh"

#include "WM_api.hh"

namespace blender::nodes::node_geo_menu_switch_cc {

NODE_STORAGE_FUNCS(NodeMenuSwitch)

static void add_input_for_enum_item(NodeDeclarationBuilder &b,
                                    const eNodeSocketDatatype type,
                                    const NodeEnumItem &enum_item)
{
  const StringRef name = enum_item.name;
  const std::string identifier = "Item" + std::to_string(enum_item.identifier);

  switch (type) {
    case SOCK_CUSTOM:
      break;
    case SOCK_FLOAT:
      b.add_input<decl::Float>(name, identifier).supports_field();
      break;
    case SOCK_VECTOR:
      b.add_input<decl::Vector>(name, identifier).supports_field();
      break;
    case SOCK_RGBA:
      b.add_input<decl::Color>(name, identifier)
          .default_value({0.8f, 0.8f, 0.8f, 1.0f})
          .supports_field();
      break;
    case SOCK_SHADER:
      b.add_input<decl::Shader>(name, identifier);
      break;
    case SOCK_BOOLEAN:
      b.add_input<decl::Bool>(name, identifier).default_value(false).supports_field();
      break;
    case SOCK_INT:
      b.add_input<decl::Int>(name, identifier).min(-100000).max(100000).supports_field();
      break;
    case SOCK_STRING:
      b.add_input<decl::String>(name, identifier).supports_field();
      break;
    case SOCK_OBJECT:
      b.add_input<decl::Object>(name, identifier);
      break;
    case SOCK_IMAGE:
      b.add_input<decl::Image>(name, identifier);
      break;
    case SOCK_GEOMETRY:
      b.add_input<decl::Geometry>(name, identifier);
      break;
    case SOCK_COLLECTION:
      b.add_input<decl::Collection>(name, identifier);
      break;
    case SOCK_TEXTURE:
      b.add_input<decl::Texture>(name, identifier);
      break;
    case SOCK_MATERIAL:
      b.add_input<decl::Material>(name, identifier);
      break;
    case SOCK_ROTATION:
      b.add_input<decl::Rotation>(name, identifier);
      break;

    case SOCK_MENU:
      /* Technically possible perhaps to select an enum based on
       * another enum, but not supported for now. */
      BLI_assert_unreachable();
      break;
  }
}

static void add_output(NodeDeclarationBuilder &b, const eNodeSocketDatatype type)
{
  StringRef name = "Output";

  switch (type) {
    case SOCK_CUSTOM:
      break;
    case SOCK_FLOAT:
      b.add_output<decl::Float>(name).dependent_field().reference_pass_all();
      break;
    case SOCK_VECTOR:
      b.add_output<decl::Vector>(name).dependent_field().reference_pass_all();
      break;
    case SOCK_RGBA:
      b.add_output<decl::Color>(name);
      break;
    case SOCK_SHADER:
      b.add_output<decl::Shader>(name);
      break;
    case SOCK_BOOLEAN:
      b.add_output<decl::Bool>(name).dependent_field().reference_pass_all();
      break;
    case SOCK_INT:
      b.add_output<decl::Int>(name).dependent_field().reference_pass_all();
      break;
    case SOCK_STRING:
      b.add_output<decl::String>(name).dependent_field().reference_pass_all();
      break;
    case SOCK_OBJECT:
      b.add_output<decl::Object>(name);
      break;
    case SOCK_IMAGE:
      b.add_output<decl::Image>(name);
      break;
    case SOCK_GEOMETRY:
      b.add_output<decl::Geometry>(name).propagate_all();
      break;
    case SOCK_COLLECTION:
      b.add_output<decl::Collection>(name);
      break;
    case SOCK_TEXTURE:
      b.add_output<decl::Texture>(name);
      break;
    case SOCK_MATERIAL:
      b.add_output<decl::Material>(name);
      break;
    case SOCK_ROTATION:
      b.add_output<decl::Rotation>(name).dependent_field().reference_pass_all();
      break;

    case SOCK_MENU:
      /* Technically possible perhaps to select an enum based on
       * another enum, but not supported for now. */
      BLI_assert_unreachable();
      break;
  }
}

static void node_declare(blender::nodes::NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }
  const NodeMenuSwitch &storage = node_storage(*node);
  const eNodeSocketDatatype data_type = eNodeSocketDatatype(storage.data_type);

  /* Remove outdated sockets. */
  b.declaration().skip_updating_sockets = false;
  /* Allow the node group interface to define the socket order. */
  b.use_custom_socket_order();

  const bool fields_type = ELEM(data_type,
                                SOCK_FLOAT,
                                SOCK_INT,
                                SOCK_BOOLEAN,
                                SOCK_VECTOR,
                                SOCK_RGBA,
                                SOCK_STRING,
                                SOCK_ROTATION);

  add_output(b, data_type);
  {
    auto sb = b.add_input<decl::Menu>("Menu").default_value(false);
    if (fields_type) {
      sb.supports_field();
    }
  }
  for (const NodeEnumItem &enum_item : storage.enum_definition.items()) {
    add_input_for_enum_item(b, data_type, enum_item);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_enum_definition_init(NodeEnumDefinition &enum_def)
{
  enum_def.next_identifier = 0;
  enum_def.items_array = nullptr;
  enum_def.items_num = 0;
}

static void node_enum_definition_free(NodeEnumDefinition &enum_def)
{
  for (NodeEnumItem &item : enum_def.items_for_write()) {
    MEM_SAFE_FREE(item.name);
    MEM_SAFE_FREE(item.description);
  }
  MEM_SAFE_FREE(enum_def.items_array);
}

static void node_enum_definition_copy(NodeEnumDefinition &dst_enum_def,
                                      const NodeEnumDefinition &src_enum_def)
{
  dst_enum_def.items_array = MEM_cnew_array<NodeEnumItem>(src_enum_def.items_num, __func__);
  dst_enum_def.items_num = src_enum_def.items_num;
  dst_enum_def.active_index = src_enum_def.active_index;
  dst_enum_def.next_identifier = src_enum_def.next_identifier;
  for (const int i : IndexRange(src_enum_def.items_num)) {
    dst_enum_def.items_array[i].identifier = src_enum_def.items_array[i].identifier;
    if (char *name = src_enum_def.items_array[i].name) {
      dst_enum_def.items_array[i].name = BLI_strdup(name);
    }
    if (char *desc = src_enum_def.items_array[i].description) {
      dst_enum_def.items_array[i].description = BLI_strdup(desc);
    }
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeMenuSwitch *data = MEM_cnew<NodeMenuSwitch>(__func__);
  node_enum_definition_init(data->enum_definition);
  data->data_type = SOCK_GEOMETRY;
  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  NodeMenuSwitch &storage = node_storage(*node);
  node_enum_definition_free(storage.enum_definition);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeMenuSwitch &src_storage = node_storage(*src_node);
  NodeMenuSwitch *dst_storage = MEM_cnew<NodeMenuSwitch>(__func__);

  node_enum_definition_copy(dst_storage->enum_definition, src_storage.enum_definition);
  dst_storage->data_type = src_storage.data_type;

  dst_node->storage = dst_storage;
}

static void node_update(bNodeTree * /*ntree*/, bNode * /*node*/) {}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (params.in_out() == SOCK_OUT) {
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeMenuSwitch");
      node_storage(node).data_type = params.socket.type;
      params.update_and_connect_available_socket(node, "Output");
    });
  }
  else {
    /* No sensible way to connect inputs currently:
     * Switch socket connection will always create a conflicting enum ref.
     * Case input sockets don't existing without an actual enum definition.
     */
  }
}

/* Multifunction which evaluates the switch input for each enum item and partially fills the output
 * array with values from the input array where the identifier matches. */
template<typename T> class MenuSwitchFn : public mf::MultiFunction {
  const NodeEnumDefinition &enum_def_;
  mf::Signature signature_;

 public:
  MenuSwitchFn(const NodeEnumDefinition &enum_def) : enum_def_(enum_def)
  {
    mf::SignatureBuilder builder{"Menu Switch", signature_};
    builder.single_input<int>("Switch");
    for (const NodeEnumItem enum_item : enum_def.items()) {
      builder.single_input<T>(enum_item.name);
    }
    builder.single_output<T>("Output");

    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const
  {
    MutableSpan<T> outputs = params.uninitialized_single_output<T>(enum_def_.items_num + 1,
                                                                   "Output");
    /* Fill outputs with the default value, so that any index without a match uses the default.
     * This causes duplicate writes but is still cheaper than keeping track of which indices are
     * uninitialized in the end. */
    outputs.fill(*static_cast<const T *>(CPPType::get<T>().default_value()));

    VArray<int> conditions = params.readonly_single_input<int>(0, "Switch");
    devirtualize_varray(conditions, [&](const auto conditions) {
      for (const int enum_i : IndexRange(enum_def_.items_num)) {
        const NodeEnumItem &enum_item = enum_def_.items()[enum_i];

        const VArray<T> inputs = params.readonly_single_input<T>(enum_i + 1, enum_item.name);
        devirtualize_varray(inputs, [&](const auto inputs) {
          mask.foreach_index([&](const int i) {
            const int condition = conditions[i];
            if (condition == enum_item.identifier) {
              outputs[i] = inputs[i];
            }
          });
        });
      }
    });
  }
};

class LazyFunctionForMenuSwitchNode : public LazyFunction {
 private:
  bool can_be_field_ = false;
  const NodeEnumDefinition &enum_def_;
  const CPPType *cpp_type_;
  std::unique_ptr<MultiFunction> multi_function_;

 public:
  LazyFunctionForMenuSwitchNode(const bNode &node) : enum_def_(node_storage(node).enum_definition)
  {
    const NodeMenuSwitch &storage = node_storage(node);
    const eNodeSocketDatatype data_type = eNodeSocketDatatype(storage.data_type);
    can_be_field_ = ELEM(
        data_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN, SOCK_VECTOR, SOCK_RGBA, SOCK_ROTATION);
    const bNodeSocketType *socket_type = nodeSocketTypeFind(
        nodeStaticSocketType(data_type, PROP_NONE));
    BLI_assert(socket_type != nullptr);
    cpp_type_ = socket_type->geometry_nodes_cpp_type;

    /* Construct multifunction if needed. */
    if (const bke::SocketValueVariantCPPType *value_or_field_type =
            bke::SocketValueVariantCPPType::get_from_self(*cpp_type_))
    {
      const CPPType &value_type = value_or_field_type->value;
      multi_function_ = std::unique_ptr<MultiFunction>(
          this->create_multi_function(value_type, enum_def_));
    }

    debug_name_ = node.name;
    inputs_.append_as("Switch", CPPType::get<SocketValueVariant<int>>());
    for (const NodeEnumItem &enum_item : storage.enum_definition.items()) {
      inputs_.append_as(enum_item.name, *cpp_type_, lf::ValueUsage::Maybe);
    }
    outputs_.append_as("Value", *cpp_type_);
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const SocketValueVariant<int> condition = params.get_input<SocketValueVariant<int>>(0);
    if (condition.is_field() && can_be_field_) {
      Field<int> condition_field = condition.as_field();
      if (condition_field.node().depends_on_input()) {
        this->execute_field(condition.as_field(), params);
        return;
      }
      const int condition_int = fn::evaluate_constant_field(condition_field);
      this->execute_single(condition_int, params);
      return;
    }
    this->execute_single(condition.as_value(), params);
  }

  void execute_single(const int condition, lf::Params &params) const
  {
    bool found_match = false;
    void *output_ptr = params.get_output_data_ptr(0);
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items_array[i];
      const int input_index = i + 1;
      if (enum_item.identifier == condition) {
        found_match = true;
        void *value_to_forward = params.try_get_input_data_ptr_or_request(input_index);
        if (value_to_forward == nullptr) {
          /* Try again when the value is available. */
          continue;
        }

        cpp_type_->move_construct(value_to_forward, output_ptr);
        params.output_set(0);
      }
      else {
        params.set_input_unused(input_index);
      }
    }
    /* No guarantee that the switch input matches any enum,
     * set default outputs to ensure valid state. */
    if (!found_match) {
      cpp_type_->value_initialize(output_ptr);
      params.output_set(0);
    }
  }

  void execute_field(Field<int> condition, lf::Params &params) const
  {
    /* When the condition is a non-constant field, we need all inputs. */
    Array<void *> item_value_or_field(enum_def_.items_num);
    bool all_inputs_available = true;
    for (const int i : IndexRange(enum_def_.items_num)) {
      const int input_index = i + 1;

      item_value_or_field[i] = params.try_get_input_data_ptr_or_request(input_index);
      if (item_value_or_field[i] == nullptr) {
        all_inputs_available = false;
      }
    }
    if (!all_inputs_available) {
      return;
    }

    const bke::SocketValueVariantCPPType &value_or_field_type =
        *bke::SocketValueVariantCPPType::get_from_self(*cpp_type_);

    Vector<GField> item_fields(enum_def_.items_num + 1);
    item_fields[0] = std::move(condition);
    for (const int i : IndexRange(enum_def_.items_num)) {
      item_fields[i + 1] = value_or_field_type.as_field(item_value_or_field[i]);
    }
    GField output_field{FieldOperation::Create(*multi_function_, std::move(item_fields))};

    void *output_ptr = params.get_output_data_ptr(0);
    value_or_field_type.construct_from_field(output_ptr, std::move(output_field));
    params.output_set(0);
  }

  MultiFunction *create_multi_function(const CPPType &type,
                                       const NodeEnumDefinition &enum_def) const
  {
    MultiFunction *multi_function = nullptr;
    type.to_static_type_tag<float,
                            int,
                            bool,
                            float3,
                            ColorGeometry4f,
                            std::string,
                            math::Quaternion>([&](auto type_tag) {
      using T = typename decltype(type_tag)::type;
      if constexpr (!std::is_void_v<T>) {
        multi_function = new MenuSwitchFn<T>(enum_def);
      }
    });
    return multi_function;
  }
};

/**
 * Outputs booleans that indicate which inputs of a menu switch node
 * are used. Note that it's possible that multiple inputs are used
 * when the condition is a field.
 */
class LazyFunctionForMenuSwitchSocketUsage : public lf::LazyFunction {
  const NodeEnumDefinition &enum_def_;

 public:
  LazyFunctionForMenuSwitchSocketUsage(const bNode &node)
      : enum_def_(node_storage(node).enum_definition)
  {
    debug_name_ = "Menu Switch Socket Usage";
    inputs_.append_as("Condition", CPPType::get<SocketValueVariant<int>>());
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items()[i];
      outputs_.append_as(enum_item.name, CPPType::get<bool>());
    }
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const SocketValueVariant<bool> &condition = params.get_input<SocketValueVariant<bool>>(0);
    if (condition.is_field()) {
      for (const int i : IndexRange(enum_def_.items_num)) {
        params.set_output(i, true);
      }
    }
    else {
      const int32_t value = condition.as_value();
      for (const int i : IndexRange(enum_def_.items_num)) {
        const NodeEnumItem &enum_item = enum_def_.items()[i];
        params.set_output(i, value == enum_item.identifier);
      }
    }
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

  geo_node_type_base(&ntype, GEO_NODE_MENU_SWITCH, "Menu Switch", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  node_type_storage(&ntype, "NodeMenuSwitch", node_free_storage, node_copy_storage);
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(register_node)

}  // namespace blender::nodes::node_geo_menu_switch_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_menu_switch_node_lazy_function(const bNode &node)
{
  using namespace node_geo_menu_switch_cc;
  BLI_assert(node.type == GEO_NODE_MENU_SWITCH);
  return std::make_unique<LazyFunctionForMenuSwitchNode>(node);
}

std::unique_ptr<LazyFunction> get_menu_switch_node_socket_usage_lazy_function(const bNode &node)
{
  using namespace node_geo_menu_switch_cc;
  BLI_assert(node.type == GEO_NODE_MENU_SWITCH);
  return std::make_unique<LazyFunctionForMenuSwitchSocketUsage>(node);
}

}  // namespace blender::nodes
