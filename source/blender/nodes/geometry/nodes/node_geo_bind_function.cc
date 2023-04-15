/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"

#include "BKE_node_runtime.hh"

#include "FN_closure.hh"
#include "FN_lazy_function_graph.hh"

#include "UI_interface.h"
#include "UI_resources.h"

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
    r_declaration.inputs.append(declaration_for_interface_socket(*input));
  }

  const FieldInferencingInterface &field_interface = *group->runtime->field_inferencing_interface;
  for (const int i : r_declaration.inputs.index_range()) {
    r_declaration.inputs[i]->input_field_type = field_interface.inputs[i];
  }
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  uiTemplateIDBrowse(
      layout, C, ptr, "node_tree", nullptr, nullptr, nullptr, UI_TEMPLATE_ID_FILTER_ALL, nullptr);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (params.node().id) {
    const bNodeTree &bind_tree = *reinterpret_cast<bNodeTree *>(params.node().id);
    const std::unique_ptr<GeometryNodesLazyFunctionGraphInfo> &lf_graph_info_ptr =
        bind_tree.runtime->geometry_nodes_lazy_function_graph_info;
    BLI_assert(lf_graph_info_ptr);

    fn::Closure closure(lf_graph_info_ptr->graph);
    params.set_output("Function", std::move(closure));
  }

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
