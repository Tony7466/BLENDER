/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"
#include <sstream>

namespace blender::nodes::node_fn_string_to_float_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String");
  b.add_output<decl::Float>("Value");
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto to_float_fn = mf::build::SI1_SO<std::string, float>(
      "String To Float", [](const std::string &a) {
        std::stringstream stream;
        stream << a;
        float f;
        if (!(stream >> f)) {
          f = 0.0f;
        }
        return f;
      });
  builder.set_matching_fn(&to_float_fn);
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_STRING_TO_FLOAT, "String to Float", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_to_float_cc
