/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"

#include "RNA_enum_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_evaluate_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Function>(N_("Function"));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeFunctionEvaluate *data = MEM_cnew<NodeFunctionEvaluate>(__func__);
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
}

}  // namespace blender::nodes::node_fn_evaluate_cc

void register_node_type_fn_evaluate()
{
  namespace file_ns = blender::nodes::node_fn_evaluate_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_EVALUATE, "Evaluate", NODE_CLASS_GROUP);
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  node_type_storage(
      &ntype, "NodeFunctionEvaluate", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
