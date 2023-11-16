/* SPDX-License-Identifier: GPL-2.0-or-later */
#include <unordered_set>

#include <fmt/format.h>

#include "DNA_node_types.h"
#include "node_geometry_util.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_task.hh"

#include "BKE_material.h"

#include "BLI_cpp_type.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "lexer.hh"
#include "operation.hh"
#include "expression.hh"
#include "parser.hh"
#include "evaluation_context.hh"
#include "math_processor.hh"

namespace blender::nodes::node_geo_math_expression_cc {
  NODE_STORAGE_FUNCS(NodeGeometryMathExpression)

  static void node_declare(blender::nodes::NodeDeclarationBuilder &b)
  {
    auto node = b.node_or_null();

    if (node == nullptr) {
      return;
    }

    NodeGeometryMathExpression *storage = static_cast<NodeGeometryMathExpression *>(node->storage);

    parse_var_names(storage->variables, [&b](std::string_view name) {
      if (name[0] == 'v') {
        b.add_input<decl::Vector>(name);
      } else if(name[0] == 'f') {
        b.add_input<decl::Float>(name);
      }
    });

    if(storage->output_type == GEO_NODE_MATH_EXPRESSION_OUTPUT_FLOAT) {
      b.add_output<decl::Float>("Value");
    } else if(storage->output_type == GEO_NODE_MATH_EXPRESSION_OUTPUT_VECTOR) {
      b.add_output<decl::Vector>("Value");
    }
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    // Most of the stuff here only needs to be done once after the expression text has changed.
    // The list of operations should be cached somewhere to avoid reparsing every time this function is called.

    const NodeGeometryMathExpression &storage = node_storage(params.node());
    std::unordered_set<std::string_view> vars;

    parse_var_names(storage.variables, [&vars](std::string_view name) {
      if (name[0] == 'v' || name[0] == 'f') {
        vars.insert(name);
      }
    });

    Parser parser;
    std::unique_ptr<Expression> expr;

    try {
      expr = parser.parse(storage.expression, nullptr);
    } catch(LexerError err) {
      params.error_message_add(NodeWarningType::Error, fmt::format("LexerError: column: {}, message: {}", err.index+1, err.message).c_str());
      params.set_default_remaining_outputs();
      return;
    } catch(ParserError err) {
      params.error_message_add(NodeWarningType::Error, fmt::format(TIP_("ParserError: column: {}, message: {}"), err.token.index+1, err.message).c_str());
      params.set_default_remaining_outputs();
      return;
    }

    EvaluationContext ctx([&params, &vars](std::string_view name) -> ValueKind {
      if(vars.find(name) == vars.end()) {
        throw "variable does not exist";
      }

      if(name[0] == 'v') {
        return ValueKind::VECTOR;
      } else if(name[0] == 'f') {
        return ValueKind::FLOAT;
      }

      BLI_assert_unreachable();
      return ValueKind();
    });

    try {
      ValueKind kind = expr->evaluate(ctx);

      if(kind == ValueKind::FLOAT && storage.output_type != GEO_NODE_MATH_EXPRESSION_OUTPUT_FLOAT) {
        params.error_message_add(NodeWarningType::Error, TIP_("The result of the expression (Float) does not match the ouput type of the node"));
        params.set_default_remaining_outputs();
        return;
      }

      if(kind == ValueKind::VECTOR && storage.output_type != GEO_NODE_MATH_EXPRESSION_OUTPUT_VECTOR) {
        params.error_message_add(NodeWarningType::Error, TIP_("The result of the expression (Vector) does not match the ouput type of the node"));
        params.set_default_remaining_outputs();
        return;
      }

      // Only the stuff below here actually needs to happen every time this function is called.
      MathProcessor proc(ctx.get_operations(), [&params](std::string_view name) {
        if(name[0] == 'v') {
          return Constant::make_vector(params.extract_input<blender::float3>(name));
        } else if(name[0] == 'f') {
          return Constant::make_float(params.extract_input<float>(name));
        }

        BLI_assert_unreachable();
        return Constant();
      });

      Constant c = proc.execute();

      if(storage.output_type == GEO_NODE_MATH_EXPRESSION_OUTPUT_FLOAT) {
        params.set_output("Value", c.get_float());
      } else if(storage.output_type == GEO_NODE_MATH_EXPRESSION_OUTPUT_VECTOR) {
        params.set_output("Value", c.get_vector());
      }
    } catch (EvaluationError err) {
      params.error_message_add(NodeWarningType::Error, fmt::format("EvaluationError: token: {}, message: {}", err.expression->get_token().value, err.message).c_str());
      params.set_default_remaining_outputs();
      return;
    }
  }

  static void node_layout(uiLayout *layout, bContext * /*c*/, PointerRNA *ptr)
  {
    uiItemR(layout, ptr, "variables", UI_ITEM_NONE, "Variables", ICON_NONE);
    uiItemR(layout, ptr, "output_type", UI_ITEM_NONE, "Output Type", ICON_NONE);
    uiItemR(layout, ptr, "expression", UI_ITEM_NONE, "Expression", ICON_NONE);
  }

  static void node_init(bNodeTree */*tree*/, bNode *node)
  {
    node->storage = MEM_callocN(sizeof(NodeGeometryMathExpression), __func__);
  }

  inline void node_register()
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
