/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"

#include "BKE_node_runtime.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_common.h"

#include "node_common.h"
#include "node_function_util.hh"
#include "node_function_util.hh"

namespace blender::nodes::node_fn_bind_cc {

static void node_declare(const bNodeTree &node_tree,
                         const bNode &node,
                         NodeDeclaration &r_declaration)
{
  const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node.id);
  if (!group) {
    return;
  }
  blender::nodes::node_group_declare_dynamic(node_tree, node, r_declaration);
  if (!node.id) {
    return;
  }
  if (ID_IS_LINKED(&group->id) && (group->id.tag & LIB_TAG_MISSING)) {
    return;
  }

  const FieldInferencingInterface &field_interface = *group->runtime->field_inferencing_interface;
  for (const int i : r_declaration.inputs.index_range()) {
    r_declaration.inputs[i]->input_field_type = field_interface.inputs[i];
  }
  for (const int i : r_declaration.outputs.index_range()) {
    r_declaration.outputs[i]->output_field_dependency = field_interface.outputs[i];
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
  ntype.poll_instance = node_group_poll_instance;

  node_type_size(&ntype, 140, 60, 400);
  ntype.declare_dynamic = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
