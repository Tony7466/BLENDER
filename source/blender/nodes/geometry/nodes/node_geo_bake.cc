/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "BKE_context.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_scene.h"
#include "BKE_screen.h"

#include "BLI_binary_search.hh"
#include "BLI_string_utils.h"

#include "DNA_modifier_types.h"
#include "DNA_space_types.h"

#include "ED_node.hh"

#include "MOD_nodes.hh"

#include "NOD_socket.hh"

#include "RNA_prototypes.h"

#include "WM_api.h"

#include "DEG_depsgraph_query.h"

#include "WM_api.h"

namespace blender::nodes::node_geo_bake_cc {

NODE_STORAGE_FUNCS(NodeGeometryBake);

static std::unique_ptr<SocketDeclaration> socket_declaration_for_bake_item(
    const NodeGeometryBakeItem &item,
    const eNodeSocketInOut in_out,
    const int corresponding_input = -1)
{
  const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);

  std::unique_ptr<SocketDeclaration> decl;

  auto handle_field_decl = [&](SocketDeclaration &decl) {
    if (in_out == SOCK_IN) {
      decl.input_field_type = InputSocketFieldType::IsSupported;
    }
    else {
      decl.output_field_dependency = OutputFieldDependency::ForPartiallyDependentField(
          {corresponding_input});
    }
  };

  switch (socket_type) {
    case SOCK_FLOAT:
      decl = std::make_unique<decl::Float>();
      handle_field_decl(*decl);
      break;
    case SOCK_VECTOR:
      decl = std::make_unique<decl::Vector>();
      handle_field_decl(*decl);
      break;
    case SOCK_RGBA:
      decl = std::make_unique<decl::Color>();
      handle_field_decl(*decl);
      break;
    case SOCK_BOOLEAN:
      decl = std::make_unique<decl::Bool>();
      handle_field_decl(*decl);
      break;
    case SOCK_ROTATION:
      decl = std::make_unique<decl::Rotation>();
      handle_field_decl(*decl);
      break;
    case SOCK_INT:
      decl = std::make_unique<decl::Int>();
      handle_field_decl(*decl);
      break;
    case SOCK_GEOMETRY:
      decl = std::make_unique<decl::Geometry>();
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

  decl->name = item.name ? item.name : "";
  decl->identifier = item.identifier_str();
  decl->in_out = in_out;
  return decl;
}

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  const NodeGeometryBake &storage = node_storage(node);
  for (const int i : IndexRange(storage.items_num)) {
    const NodeGeometryBakeItem &item = storage.items[i];
    r_declaration.inputs.append(socket_declaration_for_bake_item(item, SOCK_IN));
    r_declaration.outputs.append(socket_declaration_for_bake_item(item, SOCK_OUT, i));
  }
  r_declaration.inputs.append(decl::create_extend_declaration(SOCK_IN));
  r_declaration.outputs.append(decl::create_extend_declaration(SOCK_OUT));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryBake *data = MEM_cnew<NodeGeometryBake>(__func__);
  data->next_identifier = 0;
  data->items = MEM_cnew_array<NodeGeometryBakeItem>(1, __func__);
  data->items[0].name = BLI_strdup(DATA_("Geometry"));
  data->items[0].socket_type = SOCK_GEOMETRY;
  data->items[0].identifier = data->next_identifier++;
  data->items_num = 1;

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  NodeGeometryBake &storage = node_storage(*node);
  for (NodeGeometryBakeItem &item : storage.items_span()) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(storage.items);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryBake &src_storage = node_storage(*src_node);
  NodeGeometryBake *dst_storage = MEM_new<NodeGeometryBake>(__func__, src_storage);

  dst_storage->items = MEM_cnew_array<NodeGeometryBakeItem>(src_storage.items_num, __func__);
  for (const int i : IndexRange(src_storage.items_num)) {
    dst_storage->items[i] = src_storage.items[i];
    if (dst_storage->items[i].name) {
      dst_storage->items[i].name = BLI_strdup(dst_storage->items[i].name);
    }
  }
  dst_node->storage = dst_storage;
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  NodeGeometryBake &storage = node_storage(*node);
  if (link->tonode == node) {
    if (link->tosock->identifier == StringRef("__extend__")) {
      if (const NodeGeometryBakeItem *item = storage.add_item(
              link->fromsock->name, eNodeSocketDatatype(link->fromsock->type)))
      {
        update_node_declaration_and_sockets(*ntree, *node);
        link->tosock = nodeFindSocket(node, SOCK_IN, item->identifier_str().c_str());
        return true;
      }
    }
    else {
      return true;
    }
  }
  if (link->fromnode == node) {
    if (link->fromsock->identifier == StringRef("__extend__")) {
      if (const NodeGeometryBakeItem *item = storage.add_item(
              link->tosock->name, eNodeSocketDatatype(link->tosock->type)))
      {
        update_node_declaration_and_sockets(*ntree, *node);
        link->fromsock = nodeFindSocket(node, SOCK_OUT, item->identifier_str().c_str());
        return true;
      }
    }
    else {
      return true;
    }
  }
  return false;
}

static const bNode *group_node_by_name(const bNodeTree &ntree, StringRef name)
{
  for (const bNode *node : ntree.group_nodes()) {
    if (node->name == name) {
      return node;
    }
  }
  return nullptr;
}

static int32_t find_nested_node_id_in_root(SpaceNode *snode, const bNode *node)
{
  int32_t id_in_node = -1;
  const char *group_node_name = nullptr;
  LISTBASE_FOREACH_BACKWARD (const bNodeTreePath *, path, &snode->treepath) {
    const bNodeTree *ntree = path->nodetree;
    if (group_node_name) {
      node = group_node_by_name(*ntree, group_node_name);
    }
    bool found = false;
    for (const bNestedNodeRef &ref : ntree->nested_node_refs_span()) {
      if (node->is_group()) {
        if (ref.path.node_id == node->identifier && ref.path.id_in_node == id_in_node) {
          group_node_name = path->node_name;
          id_in_node = ref.id;
          found = true;
          break;
        }
      }
      else if (ref.path.node_id == node->identifier) {
        group_node_name = path->node_name;
        id_in_node = ref.id;
        found = true;
        break;
      }
    }
    if (!found) {
      return -1;
    }
  }
  return id_in_node;
}

static const bke::BakeNodeStorage *get_bake_storage(const NodesModifierData &nmd,
                                                    const int32_t bake_id)
{
  if (!nmd.runtime->bakes) {
    return nullptr;
  }
  return nmd.runtime->bakes->get_storage(bake_id);
}

static NodesModifierBake *get_bake(NodesModifierData &nmd, const int32_t bake_id)
{
  for (NodesModifierBake &bake : MutableSpan(nmd.bakes, nmd.bakes_num)) {
    if (bake.id == bake_id) {
      return &bake;
    }
  }
  return nullptr;
}

static void draw_bake_ui(uiLayout *layout,
                         Object &object,
                         NodesModifierData &nmd,
                         NodesModifierBake &bake,
                         const bke::BakeNodeStorage *bake_storage)
{
  const bool is_baked = bake_storage ? !bake_storage->states.is_empty() : false;

  PointerRNA bake_ptr;
  RNA_pointer_create(&object.id, &RNA_NodesModifierBake, &bake, &bake_ptr);

  uiLayout *settings_col = uiLayoutColumn(layout, false);
  uiLayoutSetActive(settings_col, !is_baked);
  {
    uiLayout *row = uiLayoutRow(settings_col, false);
    uiItemR(row, &bake_ptr, "bake_type", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
  }
  if (bake.bake_type == NODES_MODIFIER_BAKE_TYPE_ANIMATED) {
    uiLayout *subcol = uiLayoutColumn(settings_col, true);
    uiItemR(subcol, &bake_ptr, "frame_start", 0, "Start", ICON_NONE);
    uiItemR(subcol, &bake_ptr, "frame_end", 0, "End", ICON_NONE);
  }

  uiLayout *row = uiLayoutRow(layout, true);
  {
    PointerRNA op_ptr;
    uiItemFullO(row,
                "OBJECT_OT_geometry_node_bake",
                "Bake",
                ICON_NONE,
                nullptr,
                WM_OP_INVOKE_DEFAULT,
                0,
                &op_ptr);
    WM_operator_properties_id_lookup_set_from_id(&op_ptr, &object.id);
    RNA_string_set(&op_ptr, "modifier", nmd.modifier.name);
    RNA_int_set(&op_ptr, "bake_id", bake.id);
  }
  {
    PointerRNA op_ptr;
    uiItemFullO(row,
                "OBJECT_OT_geometry_node_bake_delete",
                "",
                ICON_TRASH,
                nullptr,
                WM_OP_INVOKE_DEFAULT,
                0,
                &op_ptr);
    WM_operator_properties_id_lookup_set_from_id(&op_ptr, &object.id);
    RNA_string_set(&op_ptr, "modifier", nmd.modifier.name);
    RNA_int_set(&op_ptr, "bake_id", bake.id);
  }

  if (is_baked) {
    const SubFrame bake_start = bake_storage->states.first().frame;
    const SubFrame bake_end = bake_storage->states.last().frame;

    if (bake_start == bake_end) {
      uiItemL(layout, "Baked still", ICON_LOCKED);
    }
    else {
      char message[1024];
      BLI_snprintf(
          message, sizeof(message), N_("Baked %d - %d"), bake_start.frame(), bake_end.frame());
      uiItemL(layout, message, ICON_LOCKED);
    }
  }
  else {
    uiItemL(layout, "Not baked", ICON_UNLOCKED);
  }
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  const bNode *node = static_cast<bNode *>(ptr->data);
  SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr) {
    return;
  }
  if (snode->id == nullptr) {
    return;
  }
  if (GS(snode->id->name) != ID_OB) {
    return;
  }
  Object *object = reinterpret_cast<Object *>(snode->id);
  ModifierData *md = BKE_object_active_modifier(object);
  if (md == nullptr || md->type != eModifierType_Nodes) {
    return;
  }
  NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);
  if (nmd.node_group != snode->nodetree) {
    return;
  }
  const int32_t nested_node_id = find_nested_node_id_in_root(snode, node);
  if (nested_node_id == -1) {
    return;
  }
  const bke::BakeNodeStorage *bake_storage = get_bake_storage(nmd, nested_node_id);
  NodesModifierBake *bake = get_bake(nmd, nested_node_id);
  if (bake == nullptr) {
    return;
  }

  draw_bake_ui(layout, *object, nmd, *bake, bake_storage);
}

static void bake_items_list_draw_item(uiList * /*ui_list*/,
                                      const bContext *C,
                                      uiLayout *layout,
                                      PointerRNA * /*idataptr*/,
                                      PointerRNA *itemptr,
                                      int /*icon*/,
                                      PointerRNA * /*active_dataptr*/,
                                      const char * /*active_propname*/,
                                      int /*index*/,
                                      int /*flt_flag*/)
{
  const NodeGeometryBakeItem &item = *static_cast<NodeGeometryBakeItem *>(itemptr->data);

  float4 color;
  const char *socket_type_idname = nodeStaticSocketType(item.socket_type, 0);
  ED_node_type_draw_color(socket_type_idname, color);

  uiLayout *row = uiLayoutRow(layout, true);
  uiTemplateNodeSocket(row, const_cast<bContext *>(C), color);
  uiItemL(layout, item.name, ICON_NONE);
}

static uiListType *create_items_ui_list()
{
  uiListType *items_list = MEM_cnew<uiListType>(__func__);
  STRNCPY(items_list->idname, "NODE_UL_bake_items");
  items_list->draw_item = bake_items_list_draw_item;
  WM_uilisttype_add(items_list);
  return items_list;
}

static void node_layout_ex(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  static const uiListType *items_list = create_items_ui_list();

  PointerRNA items_ptr;
  RNA_pointer_create(ptr->owner_id, &RNA_NodeGeometryBakeItems, ptr->data, &items_ptr);

  uiLayout *list_row = uiLayoutRow(layout, false);
  uiTemplateList(list_row,
                 C,
                 items_list->idname,
                 "",
                 ptr,
                 "bake_items",
                 ptr,
                 "active_index",
                 nullptr,
                 3,
                 5,
                 UILST_LAYOUT_DEFAULT,
                 0,
                 UI_TEMPLATE_LIST_FLAG_NONE);
}

class LazyFunctionForBakeNode : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForBakeNode(const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Bake";

    MutableSpan<int> lf_index_by_bsocket = own_lf_graph_info.mapping.lf_index_by_bsocket;

    const bNodeSocket &input_bsocket = node.input_socket(0);
    const bNodeSocket &output_bsocket = node.output_socket(0);
    const CPPType &type = CPPType::get<GeometrySet>();

    lf_index_by_bsocket[input_bsocket.index_in_tree()] = inputs_.append_and_get_index_as(
        "Geometry", type, lf::ValueUsage::Maybe);
    lf_index_by_bsocket[output_bsocket.index_in_tree()] = outputs_.append_and_get_index_as(
        "Geometry", type);
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    const GeoNodesLFUserData &user_data = *static_cast<const GeoNodesLFUserData *>(
        context.user_data);
    if (user_data.modifier_data->bakes == nullptr) {
      this->pass_through(params);
      return;
    }

    const Depsgraph *depsgraph = user_data.modifier_data->depsgraph;
    const Scene *scene = DEG_get_input_scene(depsgraph);
    const SubFrame frame = BKE_scene_ctime_get(scene);

    const int32_t nested_node_id =
        user_data.modifier_data->nested_node_id_by_compute_context.lookup_default(
            {user_data.compute_context->hash(), node_.identifier}, -1);
    if (nested_node_id == -1) {
      this->pass_through(params);
      return;
    }

    bke::BakeNodeStorage *bake_storage = user_data.modifier_data->bakes->get_storage(
        nested_node_id);
    if (bake_storage == nullptr) {
      this->pass_through(params);
      return;
    }

    if (bake_storage->current_bake_state) {
      GeometrySet *geometry = params.try_get_input_data_ptr_or_request<GeometrySet>(0);
      if (geometry == nullptr) {
        /* Wait until value is available. */
        return;
      }
      clean_geometry_for_cache(*geometry);
      bake_storage->current_bake_state->geometry.emplace(*geometry);
      params.set_output(0, std::move(*geometry));
    }
    else {
      if (bake_storage->states.is_empty()) {
        this->pass_through(params);
        return;
      }

      int state_index = binary_search::find_predicate_begin(
          bake_storage->states, [&](const bke::BakeNodeStateAtFrame &state_at_frame) {
            return state_at_frame.frame > frame;
          });
      /* Use the first state at the same or previous frame. When the current frame is before any
       * baked frame, use the first baked frame.*/
      if (state_index > 0) {
        state_index--;
      }

      const bke::BakeNodeState &state = *bake_storage->states[state_index].state;
      if (!state.geometry.has_value()) {
        params.set_output(0, GeometrySet());
        return;
      }
      params.set_output(0, *state.geometry);
    }
  }

  void pass_through(lf::Params &params) const
  {
    GeometrySet *geometry = params.try_get_input_data_ptr_or_request<GeometrySet>(0);
    if (geometry == nullptr) {
      /* Wait until value is available. */
      return;
    }
    clean_geometry_for_cache(*geometry);
    params.set_output(0, std::move(*geometry));
  }
};

class LazyFunctionForBakeNodeInputUsage : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForBakeNodeInputUsage(const bNode &node) : node_(node)
  {
    debug_name_ = "Bake Input Usage";
    outputs_.append_as("Usage", CPPType::get<bool>());
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    const bool inputs_required = this->bake_inputs_required(context);
    params.set_output(0, inputs_required);
  }

  bool bake_inputs_required(const lf::Context &context) const
  {
    const GeoNodesLFUserData &user_data = *static_cast<const GeoNodesLFUserData *>(
        context.user_data);
    if (user_data.modifier_data->bakes == nullptr) {
      return true;
    }
    const int32_t nested_node_id =
        user_data.modifier_data->nested_node_id_by_compute_context.lookup_default(
            {user_data.compute_context->hash(), node_.identifier}, -1);
    if (nested_node_id == -1) {
      return true;
    }
    bke::BakeNodeStorage *storage = user_data.modifier_data->bakes->get_storage(nested_node_id);
    if (storage == nullptr) {
      return true;
    }
    if (storage->states.is_empty()) {
      return true;
    }
    if (storage->current_bake_state) {
      return true;
    }
    return false;
  }
};

}  // namespace blender::nodes::node_geo_bake_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_bake_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNode>(node, own_lf_graph_info);
}

std::unique_ptr<LazyFunction> get_bake_node_input_usage_lazy_function(const bNode &node)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNodeInputUsage>(node);
}

}  // namespace blender::nodes

blender::Span<NodeGeometryBakeItem> NodeGeometryBake::items_span() const
{
  return blender::Span<NodeGeometryBakeItem>(items, items_num);
}

blender::MutableSpan<NodeGeometryBakeItem> NodeGeometryBake::items_span()
{
  return blender::MutableSpan<NodeGeometryBakeItem>(items, items_num);
}

bool NodeGeometryBakeItem::supports_type(const eNodeSocketDatatype type)
{
  return ELEM(type,
              SOCK_FLOAT,
              SOCK_VECTOR,
              SOCK_RGBA,
              SOCK_BOOLEAN,
              SOCK_ROTATION,
              SOCK_INT,
              SOCK_GEOMETRY);
}

std::string NodeGeometryBakeItem::identifier_str() const
{
  return "Item_" + std::to_string(this->identifier);
}

NodeGeometryBakeItem *NodeGeometryBake::add_item(const char *name, const eNodeSocketDatatype type)
{
  if (!NodeGeometryBakeItem::supports_type(type)) {
    return nullptr;
  }
  const int insert_index = this->items_num;
  NodeGeometryBakeItem *old_items = this->items;

  this->items = MEM_cnew_array<NodeGeometryBakeItem>(this->items_num + 1, __func__);
  std::copy_n(old_items, insert_index, this->items);
  NodeGeometryBakeItem &new_item = this->items[insert_index];
  std::copy_n(old_items + insert_index + 1,
              this->items_num - insert_index,
              this->items + insert_index + 1);

  new_item.identifier = this->next_identifier++;
  this->set_item_name(new_item, name);
  new_item.socket_type = type;

  this->items_num++;
  MEM_SAFE_FREE(old_items);
  return &new_item;
}

void NodeGeometryBake::set_item_name(NodeGeometryBakeItem &item, const char *name)
{
  char unique_name[MAX_NAME + 4];
  STRNCPY(unique_name, name);

  struct Args {
    NodeGeometryBake *storage;
    const NodeGeometryBakeItem *item;
  } args = {this, &item};

  const char *default_name = nodeStaticSocketLabel(item.socket_type, 0);
  BLI_uniquename_cb(
      [](void *arg, const char *name) {
        const Args &args = *static_cast<Args *>(arg);
        for (const NodeGeometryBakeItem &item : args.storage->items_span()) {
          if (&item != args.item) {
            if (STREQ(item.name, name)) {
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

void register_node_type_geo_bake()
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.initfunc = file_ns::node_init;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.draw_buttons_ex = file_ns::node_layout_ex;
  ntype.insert_link = file_ns::node_insert_link;
  node_type_storage(
      &ntype, "NodeGeometryBake", file_ns::node_free_storage, file_ns::node_copy_storage);
  nodeRegisterType(&ntype);
}
