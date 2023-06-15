/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_axis_angle.hh"
#include "BLI_math_euler.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_combine_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();

  b.add_input<decl::Float>("W").default_value(1.0f);
  b.add_input<decl::Float>("X").default_value(0.0f);
  b.add_input<decl::Float>("Y").default_value(0.0f);
  b.add_input<decl::Float>("Z").default_value(0.0f);

  b.add_input<decl::Vector>("Euler").subtype(PROP_EULER).make_available([](bNode &node) {
    node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ;
  });

  b.add_input<decl::Vector>("Axis")
      .default_value({0.0f, 0.0f, 1.0f})
      .make_available(
          [](bNode &node) { node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE; });
  b.add_input<decl::Float>("Angle").subtype(PROP_ANGLE).make_available([](bNode &node) {
    node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE;
  });

  b.add_input<decl::Vector>("Forward")
      .default_value({1.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXES; });
  b.add_input<decl::Vector>("Up")
      .default_value({0.0f, 0.0f, 1.0f})
      .make_available([](bNode &node) { node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXES; });

  b.add_output<decl::Rotation>("Rotation");
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ;
}

static void node_update(bNodeTree *tree, bNode *node)
{
  bke::nodeSetSocketAvailability(tree,
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->inputs, 0)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ);
  bke::nodeSetSocketAvailability(tree,
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->inputs, 1)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE);
  bke::nodeSetSocketAvailability(tree,
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->inputs, 2)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE);
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  switch (NodeCombineSeparateRotationMode(bnode.custom1)) {
    case NODE_COMBINE_SEPARATE_ROTATION_QUATERNION: {
      static auto fn = mf::build::SI4_SO<float, float, float, float, math::Quaternion>(
          "Combine Quaternion",
          [](float w, float x, float y, float z) { return math::Quaternion(w, x, y, z); });
      return &fn;
    }
    case NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE: {
      static auto fn = mf::build::SI2_SO<float3, float, math::Quaternion>(
          "Axis Angle to Quaternion", [](float3 axis, float angle) {
            return math::normalize(
                math::to_quaternion(math::AxisAngle(math::normalize(axis), angle)));
          });
      return &fn;
    }
    case NODE_COMBINE_SEPARATE_ROTATION_AXES: {
      static auto fn = mf::build::SI2_SO<float3, float3, math::Quaternion>(
          "Axes to Quaternion", [](float3 forward, float3 up) {
            const float3x3 matrix = math::from_orthonormal_axes<float3x3>(math::normalize(forward),
                                                                          math::normalize(up));
            return math::to_quaternion(matrix);
          });
      return &fn;
    }
    case NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ: {
      static auto fn = mf::build::SI1_SO<float3, math::Quaternion>(
          "Euler XYZ to Quaternion",
          [](float3 euler) { return math::to_quaternion(math::EulerXYZ(euler)); });
      return &fn;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_combine_rotation_cc

void register_node_type_fn_combine_rotation(void)
{
  namespace file_ns = blender::nodes::node_fn_combine_rotation_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_COMBINE_ROTATION, "Combine Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
