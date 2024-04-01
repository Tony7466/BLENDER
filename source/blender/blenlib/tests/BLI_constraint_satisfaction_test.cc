/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_constraint_satisfaction.hh"

namespace blender::tests {

namespace csp = constraint_satisfaction;

TEST(constraint_satisfaction, SimpleTest)
{
  /* Example taken from
   * https://www.boristhebrave.com/2021/08/30/arc-consistency-explained/
   */
  const int num_vars = 5;
  const int domain_size = 4;

  csp::ConstraintSet constraints;
  enum Symmetry {
    None,
    Symmetric,
    Antisymmetric,
  };
  auto add_binary_constraint = [&](const int a,
                                   const int b,
                                   const Symmetry symmetry,
                                   const csp::BinaryConstraintFn &constraint) {
    constraints.add(a, b, constraint);
    switch (symmetry) {
      case None:
        break;
      case Symmetric:
        constraints.add(b, a, constraint);
        break;
      case Antisymmetric: {
        const auto anti_constraint = [constraint](int value_a, int value_b) {
          return constraint(value_b, value_a);
        };
        constraints.add(b, a, anti_constraint);
        break;
      }
    }
  };

  const int var_A = 0;
  const int var_B = 1;
  const int var_C = 2;
  const int var_D = 3;
  const int var_E = 4;

  [[maybe_unused]] const int value_1 = 0;
  [[maybe_unused]] const int value_2 = 1;
  [[maybe_unused]] const int value_3 = 2;
  [[maybe_unused]] const int value_4 = 3;

  /* C = {1, 2, 4} */
  constraints.add(var_B, [](const int value) { return value != value_3; });
  /* C = {1, 3, 4} */
  constraints.add(var_C, [](const int value) { return value != value_2; });

  /* A != B */
  add_binary_constraint(var_A, var_B, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* A == D */
  add_binary_constraint(var_A, var_D, Symmetric, [](const int value_a, const int value_b) {
    return value_a == value_b;
  });
  /* A > E */
  add_binary_constraint(var_A, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* B != C */
  add_binary_constraint(var_B, var_C, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* B != D */
  add_binary_constraint(var_B, var_D, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* B > E */
  add_binary_constraint(var_B, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* C < D */
  add_binary_constraint(var_C, var_D, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a < value_b;
  });
  /* C > E */
  add_binary_constraint(var_C, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* D > E */
  add_binary_constraint(var_D, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });

  BitGroupVector<> result = csp::solve_constraints(constraints, num_vars, domain_size);

  EXPECT_FALSE(result[var_A][0]);
  EXPECT_FALSE(result[var_A][1]);
  EXPECT_FALSE(result[var_A][2]);
  EXPECT_TRUE(result[var_A][3]);

  EXPECT_FALSE(result[var_B][0]);
  EXPECT_TRUE(result[var_B][1]);
  EXPECT_FALSE(result[var_B][2]);
  EXPECT_FALSE(result[var_B][3]);

  EXPECT_FALSE(result[var_C][0]);
  EXPECT_FALSE(result[var_C][1]);
  EXPECT_TRUE(result[var_C][2]);
  EXPECT_FALSE(result[var_C][3]);

  EXPECT_FALSE(result[var_D][0]);
  EXPECT_FALSE(result[var_D][1]);
  EXPECT_FALSE(result[var_D][2]);
  EXPECT_TRUE(result[var_D][3]);

  EXPECT_TRUE(result[var_E][0]);
  EXPECT_FALSE(result[var_E][1]);
  EXPECT_FALSE(result[var_E][2]);
  EXPECT_FALSE(result[var_E][3]);
}

}  // namespace blender::tests
