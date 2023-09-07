/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "../material.h"
#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem VectorMathNodeParser::compute()
{
  /* TODO: finish some math operations */
  auto op = node_->custom1;
  NodeItem res = empty();

  /* Single operand operations */
  NodeItem x = get_input_value(0, NodeItem::Type::Any);
  switch (op) {
    case NODE_VECTOR_MATH_SINE:
      res = x.sin();
      break;
    case NODE_VECTOR_MATH_COSINE:
      res = x.cos();
      break;
    case NODE_VECTOR_MATH_TANGENT:
      res = x.tan();
      break;
    case NODE_VECTOR_MATH_ABSOLUTE:
      res = x.abs();
      break;
    case NODE_VECTOR_MATH_FLOOR:
      res = x.floor();
      break;
    case NODE_VECTOR_MATH_CEIL:
      res = x.ceil();
      break;
    case NODE_VECTOR_MATH_FRACTION:
      res = x % val(1.0f);
      break;
    case NODE_VECTOR_MATH_LENGTH:
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
      break;
    case NODE_VECTOR_MATH_NORMALIZE:
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
      break;

    default: {
      /* 2-operand operations */
      NodeItem y = get_input_value(1, NodeItem::Type::Any);
      switch (op) {
        case NODE_VECTOR_MATH_ADD:
          res = x + y;
          break;
        case NODE_VECTOR_MATH_SUBTRACT:
          res = x - y;
          break;
        case NODE_VECTOR_MATH_MULTIPLY:
          res = x * y;
          break;
        case NODE_VECTOR_MATH_DIVIDE:
          res = x / y;
          break;
        case NODE_VECTOR_MATH_MINIMUM:
          res = x.min(y);
          break;
        case NODE_VECTOR_MATH_MAXIMUM:
          res = x.max(y);
          break;
        case NODE_VECTOR_MATH_MODULO:
          res = x % y;
          break;
        case NODE_VECTOR_MATH_SNAP:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_CROSS_PRODUCT:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_DOT_PRODUCT:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_PROJECT:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_REFLECT:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_DISTANCE:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_VECTOR_MATH_SCALE:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;

        default: {
          /* 3-operand operations */
          NodeItem z = get_input_value(2, NodeItem::Type::Any);
          switch (op) {
            case NODE_VECTOR_MATH_MULTIPLY_ADD:
              res = x * y + z;
              break;
            case NODE_VECTOR_MATH_REFRACT:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;
            case NODE_VECTOR_MATH_FACEFORWARD:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;
            case NODE_VECTOR_MATH_WRAP:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;

            default:
              BLI_assert_unreachable();
          }
        }
      }
    }
  }

  return res;
}

}  // namespace blender::nodes::materialx
