/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"

namespace blender::index_mask {

struct Expr {
  enum class Type {
    Atomic,
    Union,
    Intersection,
    Difference,
  };

  Type type;

  Expr(const Type type) : type(type) {}
};

struct AtomicExpr : public Expr {
  const IndexMask *mask = nullptr;

  AtomicExpr(const IndexMask &mask) : Expr(Type::Atomic), mask(&mask) {}
};

struct UnionExpr : public Expr {
  Vector<const Expr *> terms;

  UnionExpr(const Span<const Expr *> terms) : Expr(Type::Union), terms(terms)
  {
    BLI_assert(!terms.is_empty());
    BLI_assert(!terms.contains(nullptr));
  }
};

struct IntersectionExpr : public Expr {
  Vector<const Expr *> terms;

  IntersectionExpr(const Span<const Expr *> terms) : Expr(Type::Intersection), terms(terms)
  {
    BLI_assert(!terms.is_empty());
    BLI_assert(!terms.contains(nullptr));
  }
};

struct DifferenceExpr : public Expr {
  const Expr *main_term;
  Vector<const Expr *> subtract_terms;

  DifferenceExpr(const Expr &main_term, const Span<const Expr *> subtract_terms)
      : Expr(Type::Difference), main_term(&main_term), subtract_terms(subtract_terms)
  {
    BLI_assert(!subtract_terms.contains(nullptr));
  }
};

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory);

}  // namespace blender::index_mask
