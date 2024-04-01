/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_span_ops.hh"
#include "BLI_constraint_satisfaction.hh"
#include "BLI_stack.hh"

namespace blender::constraint_satisfaction {

/* Explicit template instantiations for known logger types. */
template BitGroupVector<> solve_constraints_with_logger<NullLogger>(
    const ConstraintSet &constraints,
    const int num_vars,
    const int domain_size,
    NullLogger &logger);
template BitGroupVector<> solve_constraints_with_logger<PrintLogger>(
    const ConstraintSet &constraints,
    const int num_vars,
    const int domain_size,
    PrintLogger &logger);
template BitGroupVector<> solve_constraints_with_logger<JSONLogger>(
    const ConstraintSet &constraints,
    const int num_vars,
    const int domain_size,
    JSONLogger &logger);

/* Remove all domain values that are not allowed by the constraint. */
static void reduce_unary(const UnaryConstraintFn &constraint, MutableBitSpan domain)
{
  for (const int i : domain.index_range()) {
    if (!domain[i]) {
      continue;
    }
    if (!constraint(i)) {
      domain[i].reset();
    }
  }
}

/* Remove all domain values from A that can not be paired with any value in B. */
static bool reduce_binary(const BinaryConstraintFn &constraint,
                          BitSpan domain_src,
                          MutableBitSpan domain_dst)
{
  bool changed = false;
  for (const int i : domain_dst.index_range()) {
    if (!domain_dst[i]) {
      continue;
    }
    bool valid = false;
    for (const int j : domain_src.index_range()) {
      if (domain_src[j] && constraint(i, j)) {
        valid = true;
      }
    }
    if (!valid) {
      domain_dst[i].reset();
      changed = true;
    }
  }
  return changed;
}

/* Apply all unitary constraints. */
template<typename Logger = NullLogger>
static void solve_unary_constraints(const ConstraintSet &constraints,
                                    BitGroupVector<> &variable_domains,
                                    Logger & /*logger*/)
{
  for (const int i : variable_domains.index_range()) {
    const VariableIndex var(i);
    for (const UnaryConstraintFn &constraint : constraints.get_unary_constraints(var)) {
      reduce_unary(constraint, variable_domains[i]);
    }
  }
}

template<typename Logger = NullLogger>
static void solve_binary_constraints(const ConstraintSet &constraints,
                                     BitGroupVector<> &variable_domains,
                                     Logger &logger)
{
  /* TODO sorting the worklist could have significant impact on performance
   * by reducing unnecessary repetition of constraints.
   * Using the topological sorting of sockets should make a decent "preconditioner".
   * This is similar to what the current R-L/L-R solver does. */
  struct BinaryKey {
    VariableIndex source;
    VariableIndex target;
  };
  Stack<BinaryKey> worklist;
  logger.notify("Binary Constraint Solve");
  for (const int i : variable_domains.index_range()) {
    for (const ConstraintSet::Target &target : constraints.get_target_constraints(i)) {
      worklist.push({i, target.variable});
      logger.on_worklist_extended(i, target.variable);
    }
  }

  while (!worklist.is_empty()) {
    const BinaryKey key = worklist.pop();
    logger.on_binary_constraint_applied(key.source, key.target);
    const BinaryConstraintFn &constraint = constraints.get_binary_constraint(key.source,
                                                                             key.target);
    const BitSpan domain_src = variable_domains[key.source];
    const MutableBitSpan domain_dst = variable_domains[key.target];
    if (reduce_binary(constraint, domain_src, domain_dst)) {
      logger.on_domain_reduced(key.source, domain_src);
      if (!bits::any_bit_set(domain_src)) {
        /* TODO FAILURE CASE! */
        logger.on_domain_empty(key.target);
        break;
      }

      /* Add arcs from target to all dependant variables (except the source). */
      for (const ConstraintSet::Target &target : constraints.get_target_constraints(key.target)) {
        if (target.variable == key.source) {
          continue;
        }
        logger.on_worklist_extended(key.target, target.variable);
        worklist.push({key.source, target.variable});
      }
    }
  }
}

template<typename Logger>
BitGroupVector<> solve_constraints_with_logger(const ConstraintSet &constraints,
                                               const int num_vars,
                                               const int domain_size,
                                               Logger &logger)
{
  BitGroupVector variable_domains(num_vars, domain_size, true);

  logger.on_solve_start();
  for (const int i : variable_domains.index_range()) {
    logger.on_domain_init(i, variable_domains[i]);
  }

  solve_unary_constraints<Logger>(constraints, variable_domains, logger);
  solve_binary_constraints<Logger>(constraints, variable_domains, logger);
  logger.on_solve_end();

  return variable_domains;
}

BitGroupVector<> solve_constraints(const ConstraintSet &constraints,
                                   const int num_vars,
                                   const int domain_size)
{
  NullLogger logger;
  return solve_constraints_with_logger<NullLogger>(constraints, num_vars, domain_size, logger);
}

/* -------------------------------------------------------------------- */
/** \name Logging classes for debugging
 *
 * \{ */

void NullLogger::on_start(StringRef /*message*/) {}

void NullLogger::on_end() {}

void NullLogger::declare_variables(const int /*num_vars*/,
                                   FunctionRef<std::string(VariableIndex)> /*names_fn*/)
{
}

void NullLogger::declare_constraints(const ConstraintSet & /*constraints*/) {}

void NullLogger::notify(StringRef /*message*/) {}

void NullLogger::on_solve_start() {}

void NullLogger::on_worklist_extended(VariableIndex /*src*/, VariableIndex /*dst*/) {}

void NullLogger::on_binary_constraint_applied(VariableIndex /*src*/, VariableIndex /*dst*/) {}

void NullLogger::on_domain_init(VariableIndex /*var*/, const BitSpan /*domain*/) {}

void NullLogger::on_domain_reduced(VariableIndex /*var*/, const BitSpan /*domain*/) {}

void NullLogger::on_domain_empty(VariableIndex /*var*/) {}

void NullLogger::on_solve_end() {}

void PrintLogger::on_start(StringRef message)
{
  std::cout << message << std::endl;
}
void PrintLogger::on_end() {}

void PrintLogger::declare_variables(const int /*num_vars*/,
                        FunctionRef<std::string(VariableIndex)> /*names_fn*/)
{
}
void PrintLogger::declare_constraints(const ConstraintSet & /*constraints*/) {}

void PrintLogger::notify(StringRef message)
{
  std::cout << message << std::endl;
}

void PrintLogger::on_solve_start() {}

void PrintLogger::on_worklist_extended(VariableIndex src, VariableIndex dst)
{
  std::cout << "  Worklist extended: " << src << ", " << dst << std::endl;
}

void PrintLogger::on_binary_constraint_applied(VariableIndex src, VariableIndex dst)
{
  std::cout << "  Applying " << src << ", " << dst << std::endl;
}

void PrintLogger::on_domain_init(VariableIndex /*var*/, const BitSpan /*domain*/)
{
  std::cout << "    Initialized domain" << std::endl;
}

void PrintLogger::on_domain_reduced(VariableIndex /*var*/, const BitSpan /*domain*/)
{
  std::cout << "    Reduced domain!" << std::endl;
}

void PrintLogger::on_domain_empty(VariableIndex var)
{
  std::cout << "      FAILED! No possible values for " << var << std::endl;
}

void PrintLogger::on_solve_end() {}

  JSONLogger::JSONLogger(std::ostream &stream) : stream(stream)
{
  this->stream << "{" << std::endl;
}
JSONLogger::~JSONLogger()
{
  this->stream << "}" << std::endl;
}

template<typename Fn> void JSONLogger::on_event(StringRef type, Fn fn)
{
  if (has_events) {
    this->stream << ", ";
  }
  this->stream << "{\"type\": \"" << type << "\", ";
  fn();
  this->stream << "}" << std::endl;
  has_events = true;
}

std::string JSONLogger::stringify(StringRef s)
{
  return "\"" + s + "\"";
}

int JSONLogger::domain_as_int(const BitSpan domain)
{
  int d = 0;
  for (const int k : domain.index_range()) {
    if (domain[k]) {
      d |= (1 << k);
    }
  }
  return d;
}

void JSONLogger::declare_variables(const int num_vars,
                                    FunctionRef<std::string(VariableIndex)> names_fn)
{
  variable_names.reinitialize(num_vars);
  this->stream << "\"variables\": [";
  for (const int i : IndexRange(num_vars)) {
    variable_names[i] = names_fn(i);
    if (i > 0) {
      this->stream << ", ";
    }
    this->stream << "{\"name\": " << stringify(variable_names[i]) << "}" << std::endl;
  }
  this->stream << "]" << std::endl;
}

void JSONLogger::declare_constraints(const ConstraintSet &constraints)
{
  bool has_constraints = false;
  this->stream << ", \"constraints\": [";
  for (const auto &item : constraints.binary_constraints_by_source().items()) {
    const VariableIndex src = item.key;
    for (const ConstraintSet::Target &target : item.value) {
      const VariableIndex dst = target.variable;
      if (has_constraints) {
        this->stream << ", ";
      }
      const std::string src_name = variable_names[src];
      const std::string dst_name = variable_names[dst];
      this->stream << "{\"name\": " << stringify(src_name + "--" + dst_name)
                    << ", \"source\": " << stringify(src_name)
                    << ", \"target\": " << stringify(dst_name) << "}" << std::endl;
      has_constraints = true;
    }
  }
  this->stream << "]" << std::endl;
}

void JSONLogger::notify(StringRef /*message*/) {}

void JSONLogger::on_solve_start()
{
  this->stream << ", \"events\": [";
}

void JSONLogger::on_worklist_extended(VariableIndex src, VariableIndex dst)
{
  on_event("worklist added", [&]() {
    this->stream << "\"source\": " << stringify(variable_names[src])
                  << ", \"target\": \"" << variable_names[dst] << "\"";
  });
}

void JSONLogger::on_binary_constraint_applied(VariableIndex src, VariableIndex dst)
{
  on_event("constraint applied", [&]() {
    this->stream << "\"source\": " << stringify(variable_names[src])
                  << ", \"target\": " << stringify(variable_names[dst]);
  });
}

void JSONLogger::on_domain_init(VariableIndex var, const BitSpan domain)
{
  on_event("domain init", [&]() {
    this->stream << "\"variable\": " << stringify(variable_names[var])
                  << ", \"domain\": " << domain_as_int(domain);
  });
}

void JSONLogger::on_domain_reduced(VariableIndex var, const BitSpan domain)
{
  on_event("domain reduced", [&]() {
    this->stream << "\"variable\": " << stringify(variable_names[var])
                  << ", \"domain\": " << domain_as_int(domain);
  });
}

void JSONLogger::on_domain_empty(VariableIndex var)
{
  on_event("domain empty", [&]() {
    this->stream << "\"variable\": " << stringify(variable_names[var]);
  });
}

void JSONLogger::on_solve_end()
{
  this->stream << "]" << std::endl;
}

/** \} */

}  // namespace blender::constraint_satisfaction
