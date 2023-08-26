/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_noise.hh"

#include "node_function_util.hh"

#include "BLI_math_quaternion.hh"

namespace blender::nodes::node_fn_random_rotation_cc {

NODE_STORAGE_FUNCS(NodeRandomValue)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Int>("ID").implicit_field(implicit_field_inputs::id_or_index);
  b.add_input<decl::Int>("Seed").default_value(0).min(-10000).max(10000).supports_field();

  b.add_output<decl::Rotation>("Rotation").dependent_field();
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<int, int, math::Quaternion>(
    "Random Rotation",
    [](int id, int seed) -> math::Quaternion {
      const float s = noise::hash_to_float(seed, id, 0);
      const float sigma1 = math::sqrt(1.0f - s);
      const float sigma2 = math::sqrt(s);
      const float theta1 = M_PI * 2 * noise::hash_to_float(seed, id, 1);
      const float theta2 = M_PI * 2 * noise::hash_to_float(seed, id, 2);
      const float w = math::cos(theta2) * sigma2;
      const float x = math::sin(theta1) * sigma1;
      const float y = math::cos(theta1) * sigma1;
      const float z = math::sin(theta2) * sigma2;
      return math::Quaternion(w, x, y, z);
    },
    mf::build::exec_presets::SomeSpanOrSingle<2>()
  );
  builder.set_matching_fn(fn);
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_RANDOM_ROTATION, "Random Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  node_type_storage(
    &ntype, "NodeRandomRotation", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_random_value_cc
