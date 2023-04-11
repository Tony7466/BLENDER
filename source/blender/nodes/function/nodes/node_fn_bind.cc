/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"

#include "BKE_node_runtime.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_common.h"
#include "NOD_node_declaration.hh"

#include "node_common.h"
#include "node_function_util.hh"
#include "node_function_util.hh"

namespace blender::nodes::node_fn_bind_cc {

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

}  // namespace blender::nodes::node_fn_bind_cc

void register_node_type_fn_bind()
{
  namespace file_ns = blender::nodes::node_fn_bind_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_BIND, "Bind", NODE_CLASS_GROUP);
  ntype.declare_dynamic = file_ns::node_declare;
  ntype.poll_instance = node_group_poll_instance;
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
