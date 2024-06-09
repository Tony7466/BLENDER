/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_euler.hh"

#include "NOD_inverse_eval.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_euler_to_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Euler").subtype(PROP_EULER);
  b.add_output<decl::Rotation>("Rotation");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI1_SO<float3, math::Quaternion>(
      "Euler XYZ to Quaternion",
      [](float3 euler) { return math::to_quaternion(math::EulerXYZ(euler)); });
  builder.set_matching_fn(fn);
}

static void node_eval_inverse_elem(inverse_eval::InverseElemEvalParams &params)
{
  using namespace inverse_eval;
  const RotationElem rotation_elem = params.get_output_elem<RotationElem>("Rotation");
  VectorElem vector_elem = rotation_elem.euler;
  if (!rotation_elem.only_euler_angles()) {
    vector_elem = VectorElem::all();
  }
  params.set_input_elem("Euler", vector_elem);
}

static void node_eval_inverse(inverse_eval::InverseEvalParams &params)
{
  const math::Quaternion rotation = params.get_output<math::Quaternion>("Rotation");
  params.set_input("Euler", float3(math::to_euler(rotation)));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_EULER_TO_ROTATION, "Euler to Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  ntype.eval_inverse_elem = node_eval_inverse_elem;
  ntype.eval_inverse = node_eval_inverse;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_euler_to_rotation_cc
