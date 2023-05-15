/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_axis_angle.hh"
#include "BLI_math_euler.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_separate_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Rotation>(N_("Rotation"));
  b.add_output<decl::Vector>(N_("Euler")).subtype(PROP_EULER).make_available([](bNode &node) {
    node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ;
  });
  b.add_output<decl::Vector>(N_("Axis")).make_available([](bNode &node) {
    node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE;
  });
  b.add_output<decl::Float>(N_("Angle")).subtype(PROP_ANGLE).make_available([](bNode &node) {
    node.custom1 = NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE;
  });
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
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->outputs, 0)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ);
  bke::nodeSetSocketAvailability(tree,
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->outputs, 1)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE);
  bke::nodeSetSocketAvailability(tree,
                                 static_cast<bNodeSocket *>(BLI_findlink(&node->outputs, 2)),
                                 node->custom1 == NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE);
}

class QuaterniontoAxisAngleFunction : public mf::MultiFunction {
 public:
  QuaterniontoAxisAngleFunction()
  {
    static mf::Signature signature_;
    mf::SignatureBuilder builder{"Quaternion to Axis Angle", signature_};
    builder.single_input<math::Quaternion>("Quaternion");
    builder.single_output<float3>("Axis");
    builder.single_output<float>("Angle");
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArraySpan<math::Quaternion> quaternions =
        params.readonly_single_input<math::Quaternion>(0, "Quaternion");
    MutableSpan<float3> axes = params.uninitialized_single_output<float3>(2, "Axis");
    MutableSpan<float> angles = params.uninitialized_single_output<float>(3, "Angle");
    mask.foreach_index([&](const int64_t i) {
      const math::AxisAngle axis_angle = math::to_axis_angle(quaternions[i]);
      axes[i] = axis_angle.axis();
      angles[i] = axis_angle.angle().radian();
    });
  }
};

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const auto mode = NodeCombineSeparateRotatioNMode(bnode.custom1);

  static auto euler_xyz_fn = mf::build::SI1_SO<math::Quaternion, float3>(
      "Quaternion to Euler XYZ",
      [](const math::Quaternion quaternion) { return math::to_euler(quaternion); });
  static QuaterniontoAxisAngleFunction axis_angle_fn;

  switch (mode) {
    case NODE_COMBINE_SEPARATE_ROTATION_EULER_XYZ:
      return &euler_xyz_fn;
    case NODE_COMBINE_SEPARATE_ROTATION_AXIS_ANGLE:
      return &axis_angle_fn;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_separate_rotation_cc

void register_node_type_fn_separate_rotation(void)
{
  namespace file_ns = blender::nodes::node_fn_separate_rotation_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_SEPARATE_ROTATION, "Separate Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
