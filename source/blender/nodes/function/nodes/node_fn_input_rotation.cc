/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_euler.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_set.hh"

#include "BKE_type_conversions.hh"

#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_input_rotation_cc {

NODE_STORAGE_FUNCS(NodeInputRotation)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Rotation>("Rotation");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *col = uiLayoutColumn(layout, true);
  uiItemR(col, ptr, "rotation_euler", UI_ITEM_R_EXPAND, "", ICON_NONE);
}

static const bNodeSocket *find_single_targer(const bNodeSocket *output)
{
  return output;
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  BLI_assert(node == link->fromnode);
  ntree->ensure_topology_cache();
  NodeInputRotation &storage = node_storage(*node);
  const CPPType &node_type = CPPType::get<math::Quaternion>();

  const bNodeSocket *target = find_single_targer(link->tosock);
  if (target == nullptr) {
    return true;
  }

  const CPPType &target_value_type = *target->typeinfo->base_cpp_type;

  const bke::DataTypeConversions &convert = bke::get_implicit_type_conversions();
  if (&node_type != &target_value_type) {
    if (!convert.is_convertible(node_type, target_value_type)) {
      return true;
    }
  }

  BUFFER_FOR_CPP_TYPE_VALUE(target_value_type, target_value);
  target->typeinfo->get_base_cpp_value(target->default_value, target_value);
  target_value_type.destruct(target_value);

  math::Quaternion node_value;
  convert.convert_to_uninitialized(target_value_type, node_type, target_value, &node_value);
  const math::EulerXYZ storage_value = math::to_euler(node_value);

  storage.rotation_euler[0] = storage_value.x().radian();
  storage.rotation_euler[1] = storage_value.y().radian();
  storage.rotation_euler[2] = storage_value.z().radian();

  return true;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &bnode = builder.node();
  const NodeInputRotation &node_storage = *static_cast<const NodeInputRotation *>(bnode.storage);
  const math::EulerXYZ euler_rotation(node_storage.rotation_euler[0],
                                      node_storage.rotation_euler[1],
                                      node_storage.rotation_euler[2]);
  builder.construct_and_set_matching_fn<mf::CustomMF_Constant<math::Quaternion>>(
      math::to_quaternion(euler_rotation));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeInputRotation *data = MEM_cnew<NodeInputRotation>(__func__);
  node->storage = data;
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_INPUT_ROTATION, "Rotation", 0);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  node_type_storage(
      &ntype, "NodeInputRotation", node_free_standard_storage, node_copy_standard_storage);
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  ntype.insert_link = node_insert_link;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_input_rotation_cc
