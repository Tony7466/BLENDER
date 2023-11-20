/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include "BKE_grid_types.hh"

#include "FN_field.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Value or Field Class
 *
 * Utility class that wraps a single value and a field, to simplify accessing both of the types.
 * \{ */

template<typename T> struct ValueOrField {
  using Field = fn::Field<T>;
  using GridType = typename grid_types::AttributeGrid<T>;
  using Grid = typename std::shared_ptr<GridType>;

  /** Value that is used when the field is empty. */
  T value{};
  Field field;
  Grid grid;

  ValueOrField() = default;

  ValueOrField(T value) : value(std::move(value)) {}

  ValueOrField(Field field) : field(std::move(field)) {}

  bool is_field() const
  {
    return bool(this->field);
  }

  Field as_field() const
  {
    if (this->field) {
      return this->field;
    }
    return fn::make_constant_field(this->value);
  }

  T as_value() const
  {
    if (this->field) {
      /* This returns a default value when the field is not constant. */
      return fn::evaluate_constant_field(this->field);
    }
    return this->value;
  }

  friend std::ostream &operator<<(std::ostream &stream, const ValueOrField<T> &value_or_field)
  {
    if (value_or_field.field) {
      stream << "ValueOrField<T>";
    }
    else {
      stream << value_or_field.value;
    }
    return stream;
  }
};

/** \} */

}  // namespace blender::bke
