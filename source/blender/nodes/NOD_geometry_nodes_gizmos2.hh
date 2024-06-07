/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "DNA_node_types.h"

#include "BLI_map.hh"
#include "BLI_vector.hh"

namespace blender::nodes::gizmos2 {

struct FloatElem {};
struct IntElem {};

struct VectorElem {
  bool x = false;
  bool y = false;
  bool z = false;
};

struct RotationElem {
  bool euler_x = false;
  bool euler_y = false;
  bool euler_z = false;
  VectorElem axis;
  bool angle = false;
};

struct TransformElem {
  VectorElem translation;
  RotationElem rotation;
  VectorElem scale;
};

using ValueElemVariant = std::variant<FloatElem, IntElem, VectorElem, RotationElem, TransformElem>;

struct SocketElem {
  const bNodeSocket *socket;
  ValueElemVariant elem;
};

class InverseElemEvaluationParams {
 public:
  const bNode &node;

  const ValueElemVariant *output_elem(const int output_index) const;
  void set_input_elem(const int input_index, ValueElemVariant elem);

  const FloatElem *output_elem_float(const int output_index) const;
  const VectorElem *output_elem_vector(const int output_index) const;
  const RotationElem *output_elem_rotation(const int output_index) const;
  const TransformElem *output_elem_transform(const int output_index) const;
};

void evaluate_node_inverse_elem(InverseElemEvaluationParams &params);

class InverseEvaluationParams {
 public:
  const bNode &node;

  template<typename T> T get_output(int output_index) const;
  template<typename T> T get_input(int input_index) const;
  template<typename T> void set_input(int input_index, T value);
};

void evaluate_node_inverse(InverseEvaluationParams &params);

}  // namespace blender::nodes::gizmos2
