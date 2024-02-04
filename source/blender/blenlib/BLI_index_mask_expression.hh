/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_resource_scope.hh"

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

class ExprBuilder {
 private:
  ResourceScope scope_;

 public:
  using Term = std::variant<const Expr *, const IndexMask *>;

  const UnionExpr &merge(const Term &a, const Term &b)
  {
    return scope_.construct<UnionExpr>(
        Span<const Expr *>{&this->term_to_expr(a), &this->term_to_expr(b)});
  }

  const DifferenceExpr &subtract(const Term &a, const Term &b)
  {
    return scope_.construct<DifferenceExpr>(this->term_to_expr(a),
                                            Span<const Expr *>{&this->term_to_expr(b)});
  }

  const IntersectionExpr &intersect(const Term &a, const Term &b)
  {
    return scope_.construct<IntersectionExpr>(
        Span<const Expr *>{&this->term_to_expr(a), &this->term_to_expr(b)});
  }

 private:
  const Expr &term_to_expr(const Term &term)
  {
    if (const Expr *const *expr = std::get_if<const Expr *>(&term)) {
      return **expr;
    }
    return scope_.construct<AtomicExpr>(*std::get<const IndexMask *>(term));
  }
};

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory);

}  // namespace blender::index_mask
