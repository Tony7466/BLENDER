/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_index_mask_expression.hh"
#include "BLI_strict_flags.h"

namespace blender::index_mask {

static IndexMask evaluate_with_bool_array(const Expr &expression, IndexMaskMemory &memory)
{
  switch (expression.type) {
    case Expr::Type::Atomic: {
      const auto &expr = static_cast<const AtomicExpr &>(expression);
      return *expr.mask;
    }
    case Expr::Type::Union: {
      const auto &expr = static_cast<const UnionExpr &>(expression);
      Vector<bool> values;
      for (const Expr *term : expr.terms) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, memory);
        if (values.size() < term_mask.min_array_size()) {
          values.resize(term_mask.min_array_size(), false);
        }
        term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Intersection: {
      const auto &expr = static_cast<const IntersectionExpr &>(expression);
      const Expr &first_term = *expr.terms[0];
      const IndexMask first_term_mask = evaluate_with_bool_array(first_term, memory);
      Array<bool> values(first_term_mask.min_array_size(), false);
      first_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      for (const Expr *term : expr.terms.as_span().drop_front(1)) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, memory)
                                        .complement(IndexRange(first_term_mask.min_array_size()),
                                                    memory);
        term_mask.foreach_index([&](const int64_t i) { values[i] = false; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Difference: {
      const auto &expr = static_cast<const DifferenceExpr &>(expression);
      const IndexMask main_term_mask = evaluate_with_bool_array(*expr.main_term, memory);
      Array<bool> values(main_term_mask.min_array_size(), false);
      main_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      for (const Expr *subtract_term : expr.subtract_terms) {
        const IndexMask subtract_term_mask = evaluate_with_bool_array(*subtract_term, memory);
        subtract_term_mask.foreach_index([&](const int64_t i) {
          if (i < main_term_mask.min_array_size()) {
            values[i] = false;
          }
        });
      }
      return IndexMask::from_bools(values, memory);
    }
  }
  BLI_assert_unreachable();
  return {};
}

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory)
{
  return evaluate_with_bool_array(expression, memory);
}

}  // namespace blender::index_mask
