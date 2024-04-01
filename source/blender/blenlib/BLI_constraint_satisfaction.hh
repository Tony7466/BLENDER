/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_group_vector.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"

#include <functional>
#include <iostream>

/** \file
 * \ingroup bli
 *
 * Solve constraint satisfaction problems with a finite set of states.
 */

namespace blender::constraint_satisfaction {

/* -------------------------------------------------------------------- */
/** \name Solver for constraints satisfaction problems.
 *
 * This is an implementation of the AC3 (arc consistency 3) algorithm.
 * \{ */

/** Unary constraint function, returns true if the value is allowed. */
using UnaryConstraintFn = std::function<bool(int value)>;
/** Binary constraint function, returns true if both values are compatible. */
using BinaryConstraintFn = std::function<bool(int value_a, int value_b)>;

class VariableRef {
  int index_;

 public:
  explicit VariableRef(const int index) : index_(index) {}

  int index() const
  {
    return index_;
  }

  operator int() const
  {
    return index_;
  }

  bool operator==(const VariableRef other) const
  {
    return index_ == other.index_;
  }

  uint64_t hash() const
  {
    return get_default_hash(index_);
  }
};

struct MutableVariableRef {
  int index_;

 public:
  explicit MutableVariableRef(const int index) : index_(index) {}

  int index() const
  {
    return index_;
  }

  operator int() const
  {
    return index_;
  }

  operator VariableRef() const
  {
    return VariableRef(index_);
  }

  bool operator==(const MutableVariableRef other) const
  {
    return index_ == other.index_;
  }

  uint64_t hash() const
  {
    return get_default_hash(index_);
  }
};

/**
 * Set of unary and binary constraint functions on abstract variables.
 * Each constraint determines if a given value from the domain of a variable
 * is valid. Unary constraints are based only on the variable itself.
 * Binary constraints describe a dependency on another variable's value.
 */
class ConstraintSet {
 public:
  struct Target {
    MutableVariableRef variable;
    BinaryConstraintFn constraint;
  };
  struct Source {
    VariableRef variable;
    BinaryConstraintFn constraint;
  };

 private:
  MultiValueMap<MutableVariableRef, UnaryConstraintFn> unary_;
  MultiValueMap<VariableRef, Target> binary_by_source_;
  MultiValueMap<MutableVariableRef, Source> binary_by_target_;

 public:
  const MultiValueMap<MutableVariableRef, UnaryConstraintFn> &all_unary_constraints() const
  {
    return unary_;
  }
  const MultiValueMap<VariableRef, Target> &binary_constraints_by_source() const
  {
    return binary_by_source_;
  }
  const MultiValueMap<MutableVariableRef, Source> &binary_constraints_by_target() const
  {
    return binary_by_target_;
  }

  Span<UnaryConstraintFn> get_unary_constraints(const MutableVariableRef source) const
  {
    return unary_.lookup(source);
  }
  Span<Target> get_target_constraints(const VariableRef source) const
  {
    return binary_by_source_.lookup(source);
  }
  Span<Source> get_source_constraints(const MutableVariableRef target) const
  {
    return binary_by_target_.lookup(target);
  }
  BinaryConstraintFn get_binary_constraint(const VariableRef source_key,
                                           const MutableVariableRef target_key) const
  {
    for (const Target &target : binary_by_source_.lookup(source_key)) {
      if (target.variable == target_key) {
        return target.constraint;
      }
    }
    return nullptr;
  }

  void add(const MutableVariableRef variable, UnaryConstraintFn constraint)
  {
    unary_.add(variable, constraint);
  }
  void add(const MutableVariableRef target,
           const VariableRef source,
           BinaryConstraintFn constraint)
  {
    binary_by_source_.add(source, {target, constraint});
    binary_by_target_.add(target, {source, constraint});
  }
};

BitGroupVector<> solve_constraints(const ConstraintSet &constraints,
                                   const int num_vars,
                                   const int domain_size);

template<typename Logger>
BitGroupVector<> solve_constraints_with_logger(const ConstraintSet &constraints,
                                               const int num_vars,
                                               const int domain_size,
                                               Logger &logger);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Logging classes for debugging
 *
 * \{ */

/** Default no-op logger without performance overhead. */
struct NullLogger {
  void on_start(StringRef message);
  void on_end();

  void declare_variables(const int num_vars,
                         FunctionRef<std::string(VariableRef)> names_fn);
  void declare_constraints(const ConstraintSet & constraints);
  void notify(StringRef message);
  void on_solve_start();
  void on_worklist_extended(VariableRef src, VariableRef dst);
  void on_binary_constraint_applied(VariableRef src, VariableRef dst);
  void on_domain_init(VariableRef var, const BitSpan domain);
  void on_domain_reduced(VariableRef var, const BitSpan domain);
  void on_domain_empty(VariableRef var);
  void on_solve_end();
};

/* Simple logger printing to stdout. */
struct PrintLogger {
  void on_start(StringRef message);
  void on_end();
  void declare_variables(const int num_vars, FunctionRef<std::string(VariableRef)> names_fn);
  void declare_constraints(const ConstraintSet &constraints);
  void notify(StringRef message);
  void on_solve_start();
  void on_worklist_extended(VariableRef src, VariableRef dst);
  void on_binary_constraint_applied(VariableRef src, VariableRef dst);
  void on_domain_init(VariableRef var, const BitSpan domain);
  void on_domain_reduced(VariableRef var, const BitSpan domain);
  void on_domain_empty(VariableRef var);
  void on_solve_end();
};

/* Writes variables, constraints, and events to a json file. */
struct JSONLogger {
  std::ostream &stream;
  Array<std::string> variable_names;
  bool has_events = false;

  JSONLogger(std::ostream &stream);
  ~JSONLogger();

  template<typename Fn> void on_event(StringRef type, Fn fn);
  std::string stringify(StringRef s);
  int domain_as_int(const BitSpan domain);

  void declare_variables(const int num_vars, FunctionRef<std::string(VariableRef)> names_fn);
  void declare_constraints(const ConstraintSet &constraints);
  void notify(StringRef message);
  void on_solve_start();
  void on_worklist_extended(VariableRef src, VariableRef dst);
  void on_binary_constraint_applied(VariableRef src, VariableRef dst);
  void on_domain_init(VariableRef var, const BitSpan domain);
  void on_domain_reduced(VariableRef var, const BitSpan domain);
  void on_domain_empty(VariableRef var);
  void on_solve_end();
};

/** \} */

}  // namespace blender::constraint_satisfaction
