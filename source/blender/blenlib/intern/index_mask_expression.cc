/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_index_mask_expression.hh"

namespace blender::index_mask {

static IndexMask evaluate_with_bool_array(const Expr &expression,
                                          const IndexMask &universe,
                                          IndexMaskMemory &memory)
{
  switch (expression.type) {
    case Expr::Type::Atomic: {
      const auto &expr = static_cast<const AtomicExpr &>(expression);
      return *expr.mask;
    }
    case Expr::Type::Complement: {
      const auto &expr = static_cast<const ComplementExpr &>(expression);
      const IndexMask term_mask = evaluate_with_bool_array(*expr.term, universe, memory);
      Array<bool> values(universe.min_array_size(), true);
      term_mask.foreach_index([&](const int64_t i) { values[i] = false; });
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Union: {
      const auto &expr = static_cast<const UnionExpr &>(expression);
      Array<bool> values(universe.min_array_size(), false);
      for (const Expr *term : expr.terms) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, universe, memory);
        term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Intersection: {
      const auto &expr = static_cast<const IntersectionExpr &>(expression);
      Array<bool> values(universe.min_array_size(), false);
      {
        const Expr &first_term = *expr.terms[0];
        const IndexMask first_term_mask = evaluate_with_bool_array(first_term, universe, memory);
        first_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      }
      for (const Expr *term : expr.terms.as_span().drop_front(1)) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, universe, memory);
        term_mask.foreach_index([&](const int64_t i) { values[i] &= true; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Difference: {
      const auto &expr = static_cast<const DifferenceExpr &>(expression);
      Array<bool> values(universe.min_array_size(), false);
      {
        const IndexMask main_term_mask = evaluate_with_bool_array(
            *expr.main_term, universe, memory);
        main_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      }
      for (const Expr *subtract_term : expr.subtract_terms) {
        const IndexMask subtract_term_mask = evaluate_with_bool_array(
            *subtract_term, universe, memory);
        subtract_term_mask.foreach_index([&](const int64_t i) { values[i] = false; });
      }
      return IndexMask::from_bools(values, memory);
    }
  }
  BLI_assert_unreachable();
  return {};
}

IndexMask evaluate_expression(const Expr &expression,
                              const IndexMask &universe,
                              IndexMaskMemory &memory)
{
  return evaluate_with_bool_array(expression, universe, memory);
}

}  // namespace blender::index_mask
