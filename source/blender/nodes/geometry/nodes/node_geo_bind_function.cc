/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"

#include "BKE_node_runtime.hh"

#include "FN_lazy_function_graph.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_closure.hh"
#include "NOD_common.h"
#include "NOD_node_declaration.hh"

#include "node_common.h"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bind_function_cc {

static void node_declare(const bNodeTree &node_tree,
                         const bNode &node,
                         NodeDeclaration &r_declaration)
{
  NodeDeclarationBuilder builder(r_declaration);
  builder.add_output<decl::Function>(N_("Function"));

  const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node.id);
  if (!group) {
    return;
  }

  if (ID_IS_LINKED(&group->id) && (group->id.tag & LIB_TAG_MISSING)) {
    r_declaration.skip_updating_sockets = true;
    return;
  }
  r_declaration.skip_updating_sockets = false;

  LISTBASE_FOREACH (const bNodeSocket *, input, &group->inputs) {
    SocketDeclarationPtr decl = declaration_for_interface_socket(node_tree, *input);
    /* Need single values to bind. */
    decl->input_field_type = InputSocketFieldType::None;
    r_declaration.inputs.append(std::move(decl));
  }
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  uiTemplateIDBrowse(
      layout, C, ptr, "node_tree", nullptr, nullptr, nullptr, UI_TEMPLATE_ID_FILTER_ALL, nullptr);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (params.node().id == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }

  const bNodeTree &bind_tree = *reinterpret_cast<bNodeTree *>(params.node().id);
  const std::unique_ptr<GeometryNodesLazyFunctionGraphInfo> &lf_graph_info_ptr =
      bind_tree.runtime->geometry_nodes_lazy_function_graph_info;
  BLI_assert(lf_graph_info_ptr);

  Array<GMutablePointer> bound_values(params.node().input_sockets().size());
  for (const int i : params.node().input_sockets().index_range()) {
    const bNodeSocket *socket = params.node().input_sockets()[i];
    const CPPType *cpptype = socket->typeinfo->geometry_nodes_cpp_type;
    BLI_assert(cpptype != nullptr);

    void *bound_value_buffer = MEM_mallocN_aligned(
        cpptype->size(), cpptype->alignment(), "function bound value");
    switch (socket->type) {
      case SOCK_FLOAT: {
        ValueOrField<float> value = params.get_input<float>(socket->identifier);
        cpptype->move_construct(&value, bound_value_buffer);
        break;
      }
      case SOCK_VECTOR:
      case SOCK_RGBA:
      case SOCK_BOOLEAN:
      case SOCK_INT:
      case SOCK_STRING:
      case SOCK_OBJECT:
      case SOCK_IMAGE:
      case SOCK_GEOMETRY:
      case SOCK_COLLECTION:
      case SOCK_TEXTURE:
      case SOCK_MATERIAL:
      case SOCK_FUNCTION:
        BLI_assert_unreachable();
    }

    bound_values[i] = {cpptype, bound_value_buffer};
  }

  Closure closure(*lf_graph_info_ptr, bound_values);
  params.set_output("Function", std::move(closure));

  params.set_default_remaining_outputs();
}

}  // namespace blender::nodes::node_geo_bind_function_cc

void register_node_type_geo_bind_function()
{
  namespace file_ns = blender::nodes::node_geo_bind_function_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BIND_FUNCTION, "Bind Function", NODE_CLASS_GROUP);
  ntype.declare_dynamic = file_ns::node_declare;
  ntype.poll_instance = node_group_poll_instance;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
