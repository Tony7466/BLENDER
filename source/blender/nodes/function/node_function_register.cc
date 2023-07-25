/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_register.hh"

#include "node_function_register.hh"

void register_function_nodes()
{
  register_node_type_fn_align_euler_to_vector();
  register_node_type_fn_axis_angle_to_rotation();
  register_node_type_fn_boolean_math();
  register_node_type_fn_combine_color();
  register_node_type_fn_combine_quaternion();
  register_node_type_fn_compare();
  register_node_type_fn_euler_to_rotation();
  register_node_type_fn_float_to_int();
  register_node_type_fn_input_bool();
  register_node_type_fn_input_color();
  register_node_type_fn_input_int();
  register_node_type_fn_input_special_characters();
  register_node_type_fn_input_string();
  register_node_type_fn_input_vector();
  register_node_type_fn_invert_rotation();
  register_node_type_fn_random_value();
  register_node_type_fn_replace_string();
  register_node_type_fn_rotate_euler();
  register_node_type_fn_rotate_vector();
  register_node_type_fn_rotation_to_axis_angle();
  register_node_type_fn_rotation_to_euler();
  register_node_type_fn_separate_color();
  register_node_type_fn_separate_quaternion();
  register_node_type_fn_slice_string();
  register_node_type_fn_string_length();
  register_node_type_fn_value_to_string();
}
