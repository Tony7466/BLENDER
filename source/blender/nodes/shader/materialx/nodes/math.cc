/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"
#include "../material.h"

namespace blender::nodes::materialx {

NodeItem MathNodeParser::compute()
{
  /* TODO: finish some math operations */
  auto op = node_->custom1;
  NodeItem res = empty();

  /* Single operand operations */
  NodeItem x = get_input_value(0);
  switch (op) {
    case NODE_MATH_SINE:
      res = x.sin();
      break;
    case NODE_MATH_COSINE:
      res = x.cos();
      break;
    case NODE_MATH_TANGENT:
      res = x.tan();
      break;
    case NODE_MATH_ARCSINE:
      res = x.asin();
      break;
    case NODE_MATH_ARCCOSINE:
      res = x.acos();
      break;
    case NODE_MATH_ARCTANGENT:
      res = x.atan();
      break;
    case NODE_MATH_ROUND:
      res = (x + value(0.5f)).floor();
      break;
    case NODE_MATH_ABSOLUTE:
      res = x.abs();
      break;
    case NODE_MATH_FLOOR:
      res = x.floor();
      break;
    case NODE_MATH_CEIL:
      res = x.ceil();
      break;
    case NODE_MATH_FRACTION:
      res = x % value(1.0f);
      break;
    case NODE_MATH_SQRT:
      res = x.sqrt();
      break;
    case NODE_MATH_INV_SQRT:
      res = value(1.0f) / x.sqrt();
      break;
    case NODE_MATH_SIGN:
      res = x.sign();
      break;
    case NODE_MATH_EXPONENT:
      res = x.exp();
      break;
    case NODE_MATH_RADIANS:
      res = x * value(float(M_PI) / 180.0f);
      break;
    case NODE_MATH_DEGREES:
      res = x * value(180.0f * float(M_1_PI));
      break;
    case NODE_MATH_SINH:
      res = x.sinh();
      break;
    case NODE_MATH_COSH:
      res = x.cosh();
      break;
    case NODE_MATH_TANH:
      res = x.tanh();
      break;
    case NODE_MATH_TRUNC:
      res = x.sign() * x.abs().floor();
      break;

    default: {
      /* 2-operand operations */
      NodeItem y = get_input_value(1);
      switch (op) {
        case NODE_MATH_ADD:
          res = x + y;
          break;
        case NODE_MATH_SUBTRACT:
          res = x - y;
          break;
        case NODE_MATH_MULTIPLY:
          res = x * y;
          break;
        case NODE_MATH_DIVIDE:
          res = x / y;
          break;
        case NODE_MATH_POWER:
          res = x ^ y;
          break;
        case NODE_MATH_LOGARITHM:
          res = x.ln() / y.ln();
          break;
        case NODE_MATH_MINIMUM:
          res = x.min(y);
          break;
        case NODE_MATH_MAXIMUM:
          res = x.max(y);
          break;
        case NODE_MATH_LESS_THAN:
          res = x.if_else("<", y, value(1.0f), value(0.0f));
          break;
        case NODE_MATH_GREATER_THAN:
          res = x.if_else(">", y, value(1.0f), value(0.0f));
          break;
        case NODE_MATH_MODULO:
          res = x % y;
          break;
        case NODE_MATH_ARCTAN2:
          res = x.atan2(y);
          break;
        case NODE_MATH_SNAP:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_MATH_PINGPONG:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;
        case NODE_MATH_FLOORED_MODULO:
          CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
          break;

        default: {
          /* 3-operand operations */
          NodeItem z = get_input_value(2);
          switch (op) {
            case NODE_MATH_WRAP:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;
            case NODE_MATH_COMPARE:
              res = z.if_else("<", (x - y).abs(), value(1.0f), value(0.0f));
              break;
            case NODE_MATH_MULTIPLY_ADD:
              res = x * y + z;
              break;
            case NODE_MATH_SMOOTH_MIN:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;
            case NODE_MATH_SMOOTH_MAX:
              CLOG_WARN(LOG_MATERIALX_SHADER, "Unimplemented math operation %d", op);
              break;

            default:
              BLI_assert_unreachable();
          }
        }
      }
    }
  }

  bool clamp_output = node_->custom2 != 0;
  if (clamp_output && res) {
    res = res.clamp();
  }

  return res;
}

}  // namespace blender::nodes::materialx
