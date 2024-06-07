/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base_safe.h"
#include "BLI_math_rotation.hh"

#include "BKE_node.hh"

#include "NOD_geometry_nodes_gizmos2.hh"

namespace blender::nodes::gizmos2 {

void evaluate_node_inverse_elem(InverseElemEvaluationParams &params)
{
  switch (params.node.type) {
    case SH_NODE_MATH: {
      const NodeMathOperation operation = NodeMathOperation(params.node.custom1);
      switch (operation) {
        case NODE_MATH_ADD:
        case NODE_MATH_SUBTRACT:
        case NODE_MATH_MULTIPLY:
        case NODE_MATH_DIVIDE: {
          if (const FloatElem *output_elem = params.output_elem_float(0)) {
            params.set_input_elem(0, *output_elem);
          }
          break;
        }
      }
      break;
    }
    case SH_NODE_COMBXYZ: {
      if (const VectorElem *output_elem = params.output_elem_vector(0)) {
        if (output_elem->x) {
          params.set_input_elem(0, FloatElem());
        }
        if (output_elem->y) {
          params.set_input_elem(1, FloatElem());
        }
        if (output_elem->z) {
          params.set_input_elem(2, FloatElem());
        }
      }
      break;
    }
    case SH_NODE_SEPXYZ: {
      VectorElem result;
      result.x = params.output_elem_float(0) != nullptr;
      result.y = params.output_elem_float(1) != nullptr;
      result.z = params.output_elem_float(2) != nullptr;
      params.set_input_elem(0, result);
      break;
    }
    case FN_NODE_EULER_TO_ROTATION: {
      if (const RotationElem *output_elem = params.output_elem_rotation(0)) {
        VectorElem result;
        result.x = output_elem->euler_x;
        result.y = output_elem->euler_y;
        result.z = output_elem->euler_z;
        if (output_elem->angle || output_elem->axis.x || output_elem->axis.y ||
            output_elem->axis.z)
        {
          result.x = result.y = result.z = true;
        }
        params.set_input_elem(0, result);
      }
      break;
    }
    case FN_NODE_AXIS_ANGLE_TO_ROTATION: {
      if (const RotationElem *output_elem = params.output_elem_rotation(0)) {
        VectorElem axis_result = output_elem->axis;
        bool angle_result = output_elem->angle;
        if (output_elem->euler_x || output_elem->euler_y || output_elem->euler_z) {
          axis_result.x = axis_result.y = axis_result.z = true;
          angle_result = true;
        }
        params.set_input_elem(0, axis_result);
        if (angle_result) {
          params.set_input_elem(1, FloatElem());
        }
      }
      break;
    }
    case FN_NODE_COMBINE_TRANSFORM: {
      if (const TransformElem *output_elem = params.output_elem_transform(0)) {
        VectorElem translation = output_elem->translation;
        RotationElem rotation = output_elem->rotation;
        VectorElem scale = output_elem->scale;
        params.set_input_elem(0, translation);
        params.set_input_elem(1, rotation);
        params.set_input_elem(2, scale);
      }
      break;
    }
    case FN_NODE_SEPARATE_TRANSFORM: {
      TransformElem result;
      if (const VectorElem *output_elem = params.output_elem_vector(0)) {
        result.translation = *output_elem;
      }
      if (const RotationElem *output_elem = params.output_elem_rotation(1)) {
        result.rotation = *output_elem;
      }
      if (const VectorElem *output_elem = params.output_elem_vector(2)) {
        result.scale = *output_elem;
      }
      params.set_input_elem(0, result);
      break;
    }
    case FN_NODE_ROTATION_TO_EULER: {
      RotationElem result;
      if (const VectorElem *output_elem = params.output_elem_vector(0)) {
        result.euler_x = output_elem->x;
        result.euler_y = output_elem->y;
        result.euler_z = output_elem->z;
      }
      params.set_input_elem(0, result);
      break;
    }
    case FN_NODE_ROTATION_TO_AXIS_ANGLE: {
      RotationElem result;
      if (const VectorElem *output_elem = params.output_elem_vector(0)) {
        result.axis = *output_elem;
      }
      if (const FloatElem *output_elem = params.output_elem_float(1)) {
        result.angle = true;
      }
      params.set_input_elem(0, result);
      break;
    }
    case FN_NODE_MATRIX_MULTIPLY: {
      TransformElem result;
      /* TODO: Tag all. */
      params.set_input_elem(0, result);
      break;
    }
  }
}

void evaluate_node_inverse(InverseEvaluationParams &params)
{
  switch (params.node.type) {
    case SH_NODE_MATH: {
      const NodeMathOperation operation = NodeMathOperation(params.node.custom1);
      switch (operation) {
        case NODE_MATH_ADD: {
          const float output = params.get_output<float>(0);
          const float second_input = params.get_input<float>(1);
          const float first_input = output - second_input;
          params.set_input(0, first_input);
          break;
        }
        case NODE_MATH_SUBTRACT: {
          const float output = params.get_output<float>(0);
          const float second_input = params.get_input<float>(1);
          const float first_input = output + second_input;
          params.set_input(0, first_input);
          break;
        }
        case NODE_MATH_MULTIPLY: {
          const float output = params.get_output<float>(0);
          const float second_input = params.get_input<float>(1);
          const float first_input = safe_divide(output, second_input);
          params.set_input(0, first_input);
          break;
        }
        case NODE_MATH_DIVIDE: {
          const float output = params.get_output<float>(0);
          const float second_input = params.get_input<float>(1);
          const float first_input = output * second_input;
          params.set_input(0, first_input);
          break;
        }
      }
      break;
    }
    case SH_NODE_COMBXYZ: {
      const float3 output = params.get_output<float3>(0);
      params.set_input(0, output.x);
      params.set_input(1, output.y);
      params.set_input(2, output.z);
      break;
    }
    case SH_NODE_SEPXYZ: {
      const float3 result = {
          params.get_output<float>(0),
          params.get_output<float>(1),
          params.get_output<float>(2),
      };
      params.set_input(0, result);
      break;
    }
    case FN_NODE_EULER_TO_ROTATION: {
      const math::Quaternion output = params.get_output<math::Quaternion>(0);
      params.set_input(0, math::to_euler(output));
      break;
    }
    case FN_NODE_AXIS_ANGLE_TO_ROTATION: {
      const math::Quaternion output = params.get_output<math::Quaternion>(0);
      const math::AxisAngle axis_angle = math::to_axis_angle(output);
      params.set_input(0, axis_angle.axis());
      params.set_input(1, axis_angle.angle());
      break;
    }
    case FN_NODE_COMBINE_TRANSFORM: {
      const float4x4 output = params.get_output<float4x4>(0);
      float3 translation;
      math::Quaternion rotation;
      float3 scale;
      math::to_loc_rot_scale_safe<true>(output, translation, rotation, scale);
      params.set_input(0, translation);
      params.set_input(1, rotation);
      params.set_input(2, scale);
      break;
    }
    case FN_NODE_SEPARATE_TRANSFORM: {
      const float3 translation = params.get_output<float3>(0);
      const math::Quaternion rotation = params.get_output<math::Quaternion>(0);
      const float3 scale = params.get_output<float3>(0);
      const float4x4 result = math::from_loc_rot_scale<float4x4>(translation, rotation, scale);
      params.set_input(0, result);
      break;
    }
    case FN_NODE_ROTATION_TO_EULER: {
      const float3 euler = params.get_output<float3>(0);
      const math::Quaternion rotation = math::to_quaternion(math::EulerXYZ(euler));
      params.set_input(0, rotation);
      break;
    }
    case FN_NODE_ROTATION_TO_AXIS_ANGLE: {
      const float3 axis = params.get_output<float3>(0);
      const float angle = params.get_output<float>(1);
      const math::Quaternion rotation = math::to_quaternion(math::AxisAngle(axis, angle));
      params.set_input(0, rotation);
      break;
    }
    case FN_NODE_MATRIX_MULTIPLY: {
      const float4x4 output = params.get_output<float4x4>(0);
      const float4x4 second_input = params.get_input<float4x4>(1);
      const float4x4 first_input = output * math::invert(second_input);
      params.set_input(0, first_input);
      break;
    }
  }
}

}  // namespace blender::nodes::gizmos2
