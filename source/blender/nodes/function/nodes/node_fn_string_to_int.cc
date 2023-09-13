/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"
#include <sstream>

namespace blender::nodes::node_fn_string_to_int_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String");
  b.add_output<decl::Int>("Integer");
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto to_int_fn = mf::build::SI1_SO<std::string, int>(
      "String To Integer", [](const std::string &a) {
        std::stringstream stream;
        stream << a;
        int i;
        if (!(stream >> i)) {
          i = 0;
        }
        return i;
      });
  builder.set_matching_fn(&to_int_fn);
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_STRING_TO_INT, "String to Integer", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_to_int_cc
