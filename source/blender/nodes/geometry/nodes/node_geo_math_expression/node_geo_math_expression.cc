/* SPDX-License-Identifier: GPL-2.0-or-later */
#include <set>

#include "DNA_node_types.h"
#include "node_geometry_util.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_task.hh"

#include "BKE_material.h"

#include "BLI_cpp_type.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "lexer.h"
#include "value.h"
#include "expression.h"
#include "parser.h"
#include "evaluation_context.h"

namespace blender::nodes::node_geo_math_expression_cc {
  NODE_STORAGE_FUNCS(NodeGeometryMathExpression)

  static void node_declare(blender::nodes::NodeDeclarationBuilder &b)
  {
    auto node = b.node_or_null();

    if (node == nullptr) {
      return;
    }

    const NodeGeometryMathExpression &storage = node_storage(*node);

    Parser parser;
    std::set<std::string_view> variables;

    printf("parsing: \"%s\"\n", storage.expression);

    try {
      auto expr = parser.parse(storage.expression, &variables);

      for(auto name : variables) {
        b.add_input<decl::Float>(name);
      }
    } catch(LexerError err) {
      printf("LexerError: column: %d, message: %s\n", (int)err.index+1, err.message);
    } catch(ParserError err) {
      printf("ParserError: column: %d, message: %s\n", (int)err.token.index+1, err.message);
    }

    b.add_output<decl::Float>("Value");
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    const NodeGeometryMathExpression &storage = node_storage(params.node());

    // TODO: store the Expression generated in node_declare somewhere so re-parsing isn't needed
    Parser parser;
    EvaluationContext ctx(params);
    double value = 0.0;

    try {
      auto expr = parser.parse(storage.expression, nullptr);
      EvaluationContext ctx(params);
      value = expr->evaluate(ctx)->get_double();
    } catch(LexerError err) {
      printf("LexerError: column: %d, message: %s\n", (int)err.index+1, err.message);
    } catch(ParserError err) {
      printf("ParserError: column: %d, message: %s\n", (int)err.token.index+1, err.message);
    } catch (EvaluationError err) {
      printf("EvaluationError: token: %.*s, message: %s\n", (int)err.expression->get_token().value.size(), err.expression->get_token().value.data(), err.message);
    }

    params.set_output("Value", static_cast<float>(value));
  }

  static void node_layout(uiLayout *layout, bContext *c, PointerRNA *ptr)
  {
    uiItemR(layout, ptr, "expression", eUI_Item_Flag::UI_ITEM_R_ICON_ONLY, "Expression", ICON_NONE);
  }

  static void node_init(bNodeTree *tree, bNode *node)
  {
    node->storage = MEM_callocN(sizeof(NodeGeometryMathExpression), __func__);
    const NodeGeometryMathExpression &storage = node_storage(*node);
  }

  void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_math_expression_cc;

    static bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_MATH_EXPRESSION, "Math Expression", NODE_CLASS_CONVERTER);
    ntype.declare = file_ns::node_declare;
    ntype.initfunc = file_ns::node_init;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    ntype.draw_buttons = file_ns::node_layout;

    node_type_storage(&ntype, "NodeGeometryMathExpression", node_free_standard_storage, node_copy_standard_storage);
    nodeRegisterType(&ntype);
  }
  NOD_REGISTER_NODE(node_register)
}
