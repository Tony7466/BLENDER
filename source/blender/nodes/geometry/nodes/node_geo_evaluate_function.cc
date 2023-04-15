/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"

#include "FN_closure.hh"

#include "RNA_enum_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_common.h"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_evaluate_function_cc {

NODE_STORAGE_FUNCS(NodeGeometryEvaluateFunction)

static void node_declare(const bNodeTree &node_tree,
                         const bNode &node,
                         NodeDeclaration &r_declaration)
{
  /* May be called before storage is initialized. */
  if (node.storage == nullptr) {
    return;
  }
  const NodeGeometryEvaluateFunction &storage = node_storage(node);

  NodeDeclarationBuilder builder(r_declaration);
  builder.add_input<decl::Function>(N_("Function"));

  /* TODO define FieldInferencingInterface for this node */
  node_function_signature_declare(storage.signature, nullptr, r_declaration);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryEvaluateFunction *data = MEM_cnew<NodeGeometryEvaluateFunction>(__func__);
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
}

}  // namespace blender::nodes::node_geo_evaluate_function_cc

void register_node_type_geo_evaluate_function()
{
  namespace file_ns = blender::nodes::node_geo_evaluate_function_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_EVALUATE_FUNCTION, "Evaluate Function", NODE_CLASS_GROUP);
  ntype.declare_dynamic = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  node_type_storage(
      &ntype, "NodeGeometryEvaluateFunction", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
