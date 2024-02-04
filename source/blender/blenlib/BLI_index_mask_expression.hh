/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_resource_scope.hh"

namespace blender::index_mask {

struct AtomicExpr;
struct UnionExpr;
struct IntersectionExpr;
struct DifferenceExpr;

struct Expr {
  enum class Type {
    Atomic,
    Union,
    Intersection,
    Difference,
  };

  Type type;
  int index;

  Expr(const Type type, const int index) : type(type), index(index) {}

  int expression_array_size() const
  {
    return this->index + 1;
  }

  const AtomicExpr &as_atomic() const;
  const UnionExpr &as_union() const;
  const IntersectionExpr &as_intersection() const;
  const DifferenceExpr &as_difference() const;
};

struct AtomicExpr : public Expr {
  const IndexMask *mask = nullptr;

  AtomicExpr(const IndexMask &mask, const int index) : Expr(Type::Atomic, index), mask(&mask) {}
};

struct UnionExpr : public Expr {
  Vector<const Expr *> terms;

  UnionExpr(const Span<const Expr *> terms, const int index)
      : Expr(Type::Union, index), terms(terms)
  {
    BLI_assert(!terms.is_empty());
    BLI_assert(!terms.contains(nullptr));
  }
};

struct IntersectionExpr : public Expr {
  Vector<const Expr *> terms;

  IntersectionExpr(const Span<const Expr *> terms, const int index)
      : Expr(Type::Intersection, index), terms(terms)
  {
    BLI_assert(!terms.is_empty());
    BLI_assert(!terms.contains(nullptr));
  }
};

struct DifferenceExpr : public Expr {
  const Expr *main_term;
  Vector<const Expr *> subtract_terms;

  DifferenceExpr(const Expr &main_term, const Span<const Expr *> subtract_terms, const int index)
      : Expr(Type::Difference, index), main_term(&main_term), subtract_terms(subtract_terms)
  {
    BLI_assert(!subtract_terms.contains(nullptr));
  }
};

class ExprBuilder {
 private:
  ResourceScope scope_;
  int expr_count_ = 0;

 public:
  using Term = std::variant<const Expr *, const IndexMask *>;

  const UnionExpr &merge(const Term &a, const Term &b)
  {
    const auto &expr_a = this->term_to_expr(a);
    const auto &expr_b = this->term_to_expr(b);
    return scope_.construct<UnionExpr>(Span<const Expr *>{&expr_a, &expr_b}, expr_count_++);
  }

  const DifferenceExpr &subtract(const Term &a, const Term &b)
  {
    const auto &expr_a = this->term_to_expr(a);
    const auto &expr_b = this->term_to_expr(b);
    return scope_.construct<DifferenceExpr>(expr_a, Span<const Expr *>{&expr_b}, expr_count_++);
  }

  const IntersectionExpr &intersect(const Term &a, const Term &b)
  {
    const auto &expr_a = this->term_to_expr(a);
    const auto &expr_b = this->term_to_expr(b);
    return scope_.construct<IntersectionExpr>(Span<const Expr *>{&expr_a, &expr_b}, expr_count_++);
  }

 private:
  const Expr &term_to_expr(const Term &term)
  {
    if (const Expr *const *expr = std::get_if<const Expr *>(&term)) {
      return **expr;
    }
    return scope_.construct<AtomicExpr>(*std::get<const IndexMask *>(term), expr_count_++);
  }
};

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory);

inline const AtomicExpr &Expr::as_atomic() const
{
  BLI_assert(this->type == Type::Atomic);
  return static_cast<const AtomicExpr &>(*this);
}

inline const UnionExpr &Expr::as_union() const
{
  BLI_assert(this->type == Type::Union);
  return static_cast<const UnionExpr &>(*this);
}

inline const IntersectionExpr &Expr::as_intersection() const
{
  BLI_assert(this->type == Type::Intersection);
  return static_cast<const IntersectionExpr &>(*this);
}

inline const DifferenceExpr &Expr::as_difference() const
{
  BLI_assert(this->type == Type::Difference);
  return static_cast<const DifferenceExpr &>(*this);
}

}  // namespace blender::index_mask
