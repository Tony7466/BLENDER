/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_constraint_satisfaction.hh"

namespace blender::tests {

namespace csp = constraint_satisfaction;

TEST(constraint_satisfaction, BasicExample)
{
  /* Example taken from
   * https://www.boristhebrave.com/2021/08/30/arc-consistency-explained/
   */
  const int num_vars = 5;
  const int domain_size = 4;

  csp::ConstraintSet constraints;

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
  constraints.add_unary(var_B, [](const int value) { return value != value_3; });
  /* C = {1, 3, 4} */
  constraints.add_unary(var_C, [](const int value) { return value != value_2; });

  /* A != B */
  constraints.add_binary_symmetric(
      var_A, var_B, [](const int value_a, const int value_b) { return value_a != value_b; });
  /* A == D */
  constraints.add_binary_symmetric(
      var_A, var_D, [](const int value_a, const int value_b) { return value_a == value_b; });
  /* A > E */
  constraints.add_binary_antisymmetric(
      var_A, var_E, [](const int value_a, const int value_b) { return value_a > value_b; });
  /* B != C */
  constraints.add_binary_symmetric(
      var_B, var_C, [](const int value_a, const int value_b) { return value_a != value_b; });
  /* B != D */
  constraints.add_binary_symmetric(
      var_B, var_D, [](const int value_a, const int value_b) { return value_a != value_b; });
  /* B > E */
  constraints.add_binary_antisymmetric(
      var_B, var_E, [](const int value_a, const int value_b) { return value_a > value_b; });
  /* C < D */
  constraints.add_binary_antisymmetric(
      var_C, var_D, [](const int value_a, const int value_b) { return value_a < value_b; });
  /* C > E */
  constraints.add_binary_antisymmetric(
      var_C, var_E, [](const int value_a, const int value_b) { return value_a > value_b; });
  /* D > E */
  constraints.add_binary_antisymmetric(
      var_D, var_E, [](const int value_a, const int value_b) { return value_a > value_b; });

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

/* An over-constrained system that results in an empty variable domain. */
TEST(constraint_satisfaction, OverConstrained)
{
  const int num_vars = 3;
  const int domain_size = 2;

  csp::ConstraintSet constraints;

  const int var_A = 0;
  const int var_B = 1;
  const int var_C = 2;

  /* A = {0} */
  constraints.add_unary(var_A, [](const int value) { return value == 0; });
  /* B = {1} */
  constraints.add_unary(var_B, [](const int value) { return value == 1; });

  /* C != A */
  constraints.add_binary_symmetric(
      var_C, var_A, [](const int value_c, const int value_a) { return value_c != value_a; });
  /* C < B */
  constraints.add_binary_antisymmetric(
      var_C, var_B, [](const int value_c, const int value_b) { return value_c < value_b; });

  BitGroupVector<> result = csp::solve_constraints(constraints, num_vars, domain_size);

  EXPECT_FALSE(result[var_A][0]);
  EXPECT_FALSE(result[var_A][1]);

  EXPECT_FALSE(result[var_B][0]);
  EXPECT_TRUE(result[var_B][1]);

  EXPECT_FALSE(result[var_C][0]);
  EXPECT_FALSE(result[var_C][1]);
}

/* An under-constrained system that cannot fully resolve all variables. */
TEST(constraint_satisfaction, UnderConstrained)
{
  const int num_vars = 3;
  const int domain_size = 4;

  csp::ConstraintSet constraints;

  const int var_A = 0;
  const int var_B = 1;
  const int var_C = 2;

  /* A = {0} */
  constraints.add_unary(var_A, [](const int value) { return value == 0; });
  /* B = {3} */
  constraints.add_unary(var_B, [](const int value) { return value == 3; });

  /* C != A */
  constraints.add_binary_symmetric(
      var_C, var_A, [](const int value_c, const int value_a) { return value_c != value_a; });
  /* C < B */
  constraints.add_binary_antisymmetric(
      var_C, var_B, [](const int value_c, const int value_b) { return value_c < value_b; });

  BitGroupVector<> result = csp::solve_constraints(constraints, num_vars, domain_size);

  EXPECT_TRUE(result[var_A][0]);
  EXPECT_FALSE(result[var_A][1]);
  EXPECT_FALSE(result[var_A][2]);
  EXPECT_FALSE(result[var_A][3]);

  EXPECT_FALSE(result[var_B][0]);
  EXPECT_FALSE(result[var_B][1]);
  EXPECT_FALSE(result[var_B][2]);
  EXPECT_TRUE(result[var_B][3]);

  EXPECT_FALSE(result[var_C][0]);
  EXPECT_TRUE(result[var_C][1]);
  EXPECT_TRUE(result[var_C][2]);
  EXPECT_FALSE(result[var_C][3]);
}

TEST(constraint_satisfaction, Sudoku)
{
  const int num_vars = 81;
  const int domain_size = 9;

  csp::ConstraintSet constraints;

  /* clang-format off */
  /* Example from https://en.wikipedia.org/wiki/Sudoku
   * Values 1..9 are known states, value 0 indicates unknown state. */
  const int initial_state[9][9] = {
    { 5,  3,  0,  0,  7,  0,  0,  0,  0},
    { 6,  0,  0,  1,  9,  5,  0,  0,  0},
    { 0,  9,  8,  0,  0,  0,  0,  6,  0},
    { 8,  0,  0,  0,  6,  0,  0,  0,  3},
    { 4,  0,  0,  8,  0,  3,  0,  0,  1},
    { 7,  0,  0,  0,  2,  0,  0,  0,  6},
    { 0,  6,  0,  0,  0,  0,  2,  8,  0},
    { 0,  0,  0,  4,  1,  9,  0,  0,  5},
    { 0,  0,  0,  0,  8,  0,  0,  7,  9}};
  const int solution[9][9] = {
    { 5,  3,  4,  6,  7,  8,  9,  1,  2},
    { 6,  7,  2,  1,  9,  5,  3,  4,  8},
    { 1,  9,  8,  3,  4,  2,  5,  6,  7},
    { 8,  5,  9,  7,  6,  1,  4,  2,  3},
    { 4,  2,  6,  8,  5,  3,  7,  9,  1},
    { 7,  1,  3,  9,  2,  4,  8,  5,  6},
    { 9,  6,  1,  5,  3,  7,  2,  8,  4},
    { 2,  8,  7,  4,  1,  9,  6,  3,  5},
    { 3,  4,  5,  2,  8,  6,  1,  7,  9}};
  /* clang-format on */

  /* Inequality test for binary constraints. */
  csp::BinaryConstraintFn inequality_fn = [](int value_dst, int value_src) {
    return value_dst != value_src;
  };

  for (const int row : IndexRange(9)) {
    for (const int col : IndexRange(9)) {
      const int var = row * 9 + col;

      /* Add unary constraints for known initial states. */
      if (initial_state[row][col] > 0) {
        const int known_value = initial_state[row][col] - 1;
        constraints.add_unary(var,
                              [known_value](const int value) { return value == known_value; });
      }

      /* Sudoku rules:
       * Cell value cannot be the same as that in any other cell
       * in the same row, column, or block.
       *
       * For simplicity only one direction of each symmetric constraint is added per cell. */
      for (const int other_col : IndexRange(9)) {
        const int other_var_in_row = row * 9 + other_col;
        if (other_var_in_row != var) {
          constraints.add_binary(var, other_var_in_row, inequality_fn);
        }
      }
      for (const int other_row : IndexRange(9)) {
        const int other_var_in_col = other_row * 9 + col;
        if (other_var_in_col != var) {
          constraints.add_binary(var, other_var_in_col, inequality_fn);
        }
      }
      for (const int other_row : IndexRange(int(row / 3) * 3, 3)) {
        for (const int other_col : IndexRange(int(col / 3) * 3, 3)) {
          const int other_var_in_block = other_row * 9 + other_col;
          if (other_var_in_block != var) {
            constraints.add_binary(var, other_var_in_block, inequality_fn);
          }
        }
      }
    }
  }

  BitGroupVector<> result = csp::solve_constraints(constraints, num_vars, domain_size);

  for (const int row : IndexRange(9)) {
    for (const int col : IndexRange(9)) {
      const int var = row * 9 + col;
      const int expected_value = solution[row][col] - 1;
      for (const int value : IndexRange(9)) {
        EXPECT_EQ(result[var][value], value == expected_value);
      }
    }
  }
}

}  // namespace blender::tests
