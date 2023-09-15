/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_string.h"
#include "BLI_string_utils.h"

#include "BKE_node_tree_update.h"
#include "BKE_report.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "FN_field_cpp_type.hh"

#include "WM_api.hh"

namespace blender::nodes::node_geo_menu_switch_cc {

NODE_STORAGE_FUNCS(NodeMenuSwitch)

static void node_declare_dynamic(const bNodeTree &tree,
                                 const bNode &node,
                                 NodeDeclaration &declaration)
{
  const NodeMenuSwitch &storage = node_storage(node);
  const eNodeSocketDatatype input_type = eNodeSocketDatatype(storage.input_type);

  /* Remove outdated sockets. */
  declaration.skip_updating_sockets = false;
  /* Allow the node group interface to define the socket order. */
  declaration.use_custom_socket_order = true;

  NodeDeclarationBuilder b(declaration);

  const bool fields_type = ELEM(storage.input_type,
                                SOCK_FLOAT,
                                SOCK_INT,
                                SOCK_BOOLEAN,
                                SOCK_VECTOR,
                                SOCK_RGBA,
                                SOCK_STRING,
                                SOCK_ROTATION);

  {
    // TODO implement the Enum socket type
    auto sb = b.add_input<decl::Bool>("Switch").default_value(false);
    if (fields_type) {
      sb.supports_field();
    }
  }

  for (const NodeMenuSwitchEnumItem *enum_item : storage.items()) {
    StringRef name = enum_item->name;

    switch (input_type) {
      case SOCK_CUSTOM:
        break;
      case SOCK_FLOAT:
        b.add_input_output<decl::Float>(name)
            .supports_field()
            .dependent_field()
            .reference_pass_all();
        break;
      case SOCK_VECTOR:
        b.add_input_output<decl::Vector>(name)
            .supports_field()
            .dependent_field()
            .reference_pass_all();
        break;
      case SOCK_RGBA:
        b.add_input_output<decl::Color>(name)
            .default_value({0.8f, 0.8f, 0.8f, 1.0f})
            .supports_field();
        break;
      case SOCK_SHADER:
        b.add_input_output<decl::Shader>(name);
        break;
      case SOCK_BOOLEAN:
        b.add_input_output<decl::Bool>(name)
            .default_value(false)
            .hide_value()
            .supports_field()
            .dependent_field()
            .reference_pass_all();
        break;
      case SOCK_INT:
        b.add_input_output<decl::Int>(name)
            .min(-100000)
            .max(100000)
            .supports_field()
            .dependent_field()
            .reference_pass_all();
        break;
      case SOCK_STRING:
        b.add_input_output<decl::String>(name)
            .supports_field()
            .dependent_field()
            .reference_pass_all();
        break;
      case SOCK_OBJECT:
        b.add_input_output<decl::Object>(name);
        break;
      case SOCK_IMAGE:
        b.add_input_output<decl::Image>(name);
        break;
      case SOCK_GEOMETRY:
        b.add_input_output<decl::Geometry>(name).propagate_all();
        break;
      case SOCK_COLLECTION:
        b.add_input_output<decl::Collection>(name);
        break;
      case SOCK_TEXTURE:
        b.add_input_output<decl::Texture>(name);
        break;
      case SOCK_MATERIAL:
        b.add_input_output<decl::Material>(name);
        break;
      case SOCK_ROTATION:
        b.add_input_output<decl::Rotation>(name).dependent_field().reference_pass_all();
        break;
    }
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "input_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeMenuSwitch *data = MEM_cnew<NodeMenuSwitch>(__func__);
  data->input_type = SOCK_GEOMETRY;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node) {}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (params.in_out() == SOCK_OUT) {
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeMenuSwitch");
      node_storage(node).input_type = params.socket.type;
      params.update_and_connect_available_socket(node, "Output");
    });
  }
  else {
    /* Make sure the switch input comes first in the search for boolean sockets. */
    int true_false_weights = 0;
    if (params.other_socket().type == SOCK_BOOLEAN) {
      params.add_item(IFACE_("Switch"), [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("GeometryNodeMenuSwitch");
        params.update_and_connect_available_socket(node, "Switch");
      });
      true_false_weights--;
    }

    params.add_item(
        IFACE_("False"),
        [](LinkSearchOpParams &params) {
          bNode &node = params.add_node("GeometryNodeMenuSwitch");
          node_storage(node).input_type = params.socket.type;
          params.update_and_connect_available_socket(node, "False");
        },
        true_false_weights);
    params.add_item(
        IFACE_("True"),
        [](LinkSearchOpParams &params) {
          bNode &node = params.add_node("GeometryNodeMenuSwitch");
          node_storage(node).input_type = params.socket.type;
          params.update_and_connect_available_socket(node, "True");
        },
        true_false_weights);
  }
}

class LazyFunctionForMenuSwitchNode : public LazyFunction {
 private:
  bool can_be_field_ = false;

 public:
  LazyFunctionForMenuSwitchNode(const bNode &node)
  {
    const NodeMenuSwitch &storage = node_storage(node);
    const eNodeSocketDatatype data_type = eNodeSocketDatatype(storage.input_type);
    can_be_field_ = ELEM(
        data_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN, SOCK_VECTOR, SOCK_RGBA, SOCK_ROTATION);

    const bNodeSocketType *socket_type = nullptr;
    for (const bNodeSocket *socket : node.output_sockets()) {
      if (socket->type == data_type) {
        socket_type = socket->typeinfo;
        break;
      }
    }
    BLI_assert(socket_type != nullptr);
    const CPPType &cpp_type = *socket_type->geometry_nodes_cpp_type;

    debug_name_ = node.name;
    inputs_.append_as("Condition", CPPType::get<ValueOrField<bool>>());
    inputs_.append_as("False", cpp_type, lf::ValueUsage::Maybe);
    inputs_.append_as("True", cpp_type, lf::ValueUsage::Maybe);
    outputs_.append_as("Value", cpp_type);
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const ValueOrField<bool> condition = params.get_input<ValueOrField<bool>>(0);
    if (condition.is_field() && can_be_field_) {
      Field<bool> condition_field = condition.as_field();
      if (condition_field.node().depends_on_input()) {
        this->execute_field(condition.as_field(), params);
        return;
      }
      const bool condition_bool = fn::evaluate_constant_field(condition_field);
      this->execute_single(condition_bool, params);
      return;
    }
    this->execute_single(condition.as_value(), params);
  }

  static constexpr int false_input_index = 1;
  static constexpr int true_input_index = 2;

  void execute_single(const bool condition, lf::Params &params) const
  {
    const int input_to_forward = condition ? true_input_index : false_input_index;
    const int input_to_ignore = condition ? false_input_index : true_input_index;

    params.set_input_unused(input_to_ignore);
    void *value_to_forward = params.try_get_input_data_ptr_or_request(input_to_forward);
    if (value_to_forward == nullptr) {
      /* Try again when the value is available. */
      return;
    }

    const CPPType &type = *outputs_[0].type;
    void *output_ptr = params.get_output_data_ptr(0);
    type.move_construct(value_to_forward, output_ptr);
    params.output_set(0);
  }

  void execute_field(Field<bool> condition, lf::Params &params) const
  {
    /* When the condition is a non-constant field, we need both inputs. */
    void *false_value_or_field = params.try_get_input_data_ptr_or_request(false_input_index);
    void *true_value_or_field = params.try_get_input_data_ptr_or_request(true_input_index);
    if (ELEM(nullptr, false_value_or_field, true_value_or_field)) {
      /* Try again when inputs are available. */
      return;
    }

    const CPPType &type = *outputs_[0].type;
    const fn::ValueOrFieldCPPType &value_or_field_type = *fn::ValueOrFieldCPPType::get_from_self(
        type);
    const CPPType &value_type = value_or_field_type.value;
    const MultiFunction &switch_multi_function = this->get_switch_multi_function(value_type);

    GField false_field = value_or_field_type.as_field(false_value_or_field);
    GField true_field = value_or_field_type.as_field(true_value_or_field);

    GField output_field{FieldOperation::Create(
        switch_multi_function,
        {std::move(condition), std::move(false_field), std::move(true_field)})};

    void *output_ptr = params.get_output_data_ptr(0);
    value_or_field_type.construct_from_field(output_ptr, std::move(output_field));
    params.output_set(0);
  }

  const MultiFunction &get_switch_multi_function(const CPPType &type) const
  {
    const MultiFunction *switch_multi_function = nullptr;
    type.to_static_type_tag<float,
                            int,
                            bool,
                            float3,
                            ColorGeometry4f,
                            std::string,
                            math::Quaternion>([&](auto type_tag) {
      using T = typename decltype(type_tag)::type;
      if constexpr (std::is_void_v<T>) {
        BLI_assert_unreachable();
      }
      else {
        static auto switch_fn = mf::build::SI3_SO<bool, T, T, T>(
            "Switch", [](const bool condition, const T &false_value, const T &true_value) {
              return condition ? true_value : false_value;
            });
        switch_multi_function = &switch_fn;
      }
    });
    BLI_assert(switch_multi_function != nullptr);
    return *switch_multi_function;
  }
};

//static NodeMenuSwitchEnumItem *rna_enum_items_new(
//    ID *id, bNode *node, Main *bmain, ReportList *reports, int socket_type, const char *name)
//{
//  NodeMenuSwitch &storage = node_storage(*node);
//  NodeMenuSwitchEnumItem *item = storage.add_item(name);
//  if (item == nullptr) {
//    BKE_report(reports, RPT_ERROR, "Unable to create item");
//  }
//  else {
//    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
//    BKE_ntree_update_tag_node_property(ntree, node);
//    BKE_ntree_update_main_tree(bmain, ntree, nullptr);
//    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
//  }
//
//  return item;
//}
//
//static void rna_enum_items_remove(
//    ID *id, bNode *node, Main *bmain, ReportList *reports, NodeMenuSwitchEnumItem *item)
//{
//  NodeMenuSwitch &storage = node_storage(*node);
//  if (!storage.remove_item(*item)) {
//    BKE_reportf(reports, RPT_ERROR, "Unable to remove item '%s' from node", item->name);
//  }
//  else {
//    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
//    BKE_ntree_update_tag_node_property(ntree, node);
//    BKE_ntree_update_main_tree(bmain, ntree, nullptr);
//    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
//  }
//}
//
//static void rna_enum_items_clear(ID *id, bNode *node, Main *bmain)
//{
//  NodeMenuSwitch &storage = node_storage(*node);
//  storage.clear_items();
//
//  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
//  BKE_ntree_update_tag_node_property(ntree, node);
//  BKE_ntree_update_main_tree(bmain, ntree, nullptr);
//  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
//}
//
//static void rna_enum_items_move(
//    ID *id, bNode *node, Main *bmain, ReportList *reports, int from_index, int to_index)
//{
//  NodeMenuSwitch &storage = node_storage(*node);
//
//  if (!storage.move_item(from_index, to_index)) {
//    BKE_reportf(
//        reports, RPT_ERROR, "Unable to move item from index %d to index %d", from_index, to_index);
//    return;
//  }
//
//  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
//  BKE_ntree_update_tag_node_property(ntree, node);
//  BKE_ntree_update_main_tree(bmain, ntree, nullptr);
//  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
//}
//
//static PointerRNA rna_active_item_get(PointerRNA *ptr)
//{
//  bNode *node = static_cast<bNode *>(ptr->data);
//  const NodeMenuSwitch &storage = node_storage(*node);
//  NodeMenuSwitchEnumItem *item = storage.items().get(storage.active_index, nullptr);
//  PointerRNA r_ptr = RNA_pointer_create(ptr->owner_id, &RNA_NodeGeometryMenuSwitchEnumItem, item);
//  return r_ptr;
//}
//
//static void rna_active_item_set(PointerRNA *ptr, PointerRNA value, ReportList * /*reports*/)
//{
//  bNode *node = static_cast<bNode *>(ptr->data);
//  NodeMenuSwitch &storage = node_storage(*node);
//  NodeMenuSwitchEnumItem *item = static_cast<NodeMenuSwitchEnumItem *>(value.data);
//  storage.active_index = storage.items().first_index_try(item);
//}
//
//static void rna_enum_items_api(StructRNA *srna)
//{
//  PropertyRNA *prop;
//  PropertyRNA *parm;
//  FunctionRNA *func;
//
//  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
//  RNA_def_property_int_sdna(prop, nullptr, "active_index");
//  RNA_def_property_ui_text(prop, "Active Index", "Index of the active item");
//  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
//  RNA_def_property_update(prop, NC_NODE, nullptr);
//
//  prop = RNA_def_property(srna, "active", PROP_POINTER, PROP_NONE);
//  RNA_def_property_struct_type(prop, "NodeMenuSwitchEnumItem");
//  RNA_def_property_flag(prop, PROP_EDITABLE);
//  RNA_def_property_pointer_funcs(prop, "rna_active_get", "rna_active_set", nullptr, nullptr);
//  RNA_def_property_ui_text(prop, "Active", "Active item");
//  RNA_def_property_update(prop, NC_NODE, nullptr);
//
//  func = RNA_def_function(srna, "new", "rna_NodeGeometryRepeatOutput_items_new");
//  RNA_def_function_ui_description(func, "Add a item to this repeat zone");
//  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
//  parm = RNA_def_enum(func,
//                      "socket_type",
//                      rna_enum_node_socket_data_type_items,
//                      SOCK_GEOMETRY,
//                      "Socket Type",
//                      "Socket type of the item");
//  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
//  parm = RNA_def_string(func, "name", nullptr, MAX_NAME, "Name", "");
//  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
//  /* return value */
//  parm = RNA_def_pointer(func, "item", "RepeatItem", "Item", "New item");
//  RNA_def_function_return(func, parm);
//
//  func = RNA_def_function(srna, "remove", "rna_NodeGeometryRepeatOutput_items_remove");
//  RNA_def_function_ui_description(func, "Remove an item from this repeat zone");
//  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
//  parm = RNA_def_pointer(func, "item", "RepeatItem", "Item", "The item to remove");
//  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
//
//  func = RNA_def_function(srna, "clear", "rna_NodeGeometryRepeatOutput_items_clear");
//  RNA_def_function_ui_description(func, "Remove all items from this repeat zone");
//  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
//
//  func = RNA_def_function(srna, "move", "rna_NodeGeometryRepeatOutput_items_move");
//  RNA_def_function_ui_description(func, "Move an item to another position");
//  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
//  parm = RNA_def_int(
//      func, "from_index", -1, 0, INT_MAX, "From Index", "Index of the item to move", 0, 10000);
//  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
//  parm = RNA_def_int(
//      func, "to_index", -1, 0, INT_MAX, "To Index", "Target index for the item", 0, 10000);
//  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
//}

static void node_rna(StructRNA *srna)
{
  //PropertyRNA *prop;

  // prop = RNA_def_property(srna, "enum_items", PROP_COLLECTION, PROP_NONE);
  // RNA_def_property_collection_sdna(prop, nullptr, "items_array", "items_num");
  // RNA_def_property_collection_funcs(prop, accessors.getter, accessors.setter, item_func);
  // RNA_def_property_struct_type(prop, "NodeMenuSwitchEnumItem");
  // RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  // RNA_def_property_ui_text(prop, "Enum Items", "Declaration of enum values");
  //rna_enum_items_api(srna);

  RNA_def_node_enum(
      srna,
      "input_type",
      "Input Type",
      "",
      rna_enum_node_socket_data_type_items,
      NOD_storage_enum_accessors(input_type),
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
  ntype.declare_dynamic = node_declare_dynamic;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  node_type_storage(
      &ntype, "NodeMenuSwitch", node_free_standard_storage, node_copy_standard_storage);
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

}  // namespace blender::nodes

blender::Span<NodeEnumItem *> NodeEnumDefinition::items() const
{
  return {this->items_array, this->items_num};
}

blender::MutableSpan<NodeEnumItem *> NodeEnumDefinition::items_for_write()
{
  return {this->items_array, this->items_num};
}

void NodeEnumDefinition::set_item_name(NodeEnumItem &item, const char *name)
{
  char unique_name[MAX_NAME + 4];
  STRNCPY(unique_name, name);

  struct Args {
    NodeEnumDefinition *storage;
    const NodeEnumItem *item;
  } args = {this, &item};

  const char *default_name = items().is_empty() ? "Item" : items().last()->name;
  BLI_uniquename_cb(
      [](void *arg, const char *name) {
        const Args &args = *static_cast<Args *>(arg);
        for (const NodeEnumItem *item : args.storage->items()) {
          if (item != args.item) {
            if (STREQ(item->name, name)) {
              return true;
            }
          }
        }
        return false;
      },
      &args,
      default_name,
      '.',
      unique_name,
      ARRAY_SIZE(unique_name));

  MEM_SAFE_FREE(item.name);
  item.name = BLI_strdup(unique_name);
}
