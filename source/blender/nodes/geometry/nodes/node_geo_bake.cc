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

#include "BLI_binary_search.hh"

#include "DNA_modifier_types.h"
#include "DNA_space_types.h"

#include "MOD_nodes.hh"

#include "RNA_prototypes.h"

#include "DEG_depsgraph_query.h"

#include "WM_api.h"

namespace blender::nodes::node_geo_bake_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry");
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
    uiItemL(layout, "Baked", ICON_LOCKED);
  }
  else {
    uiItemL(layout, "Not Baked", ICON_UNLOCKED);
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

void register_node_type_geo_bake()
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
