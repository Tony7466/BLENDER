/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include "BKE_volume_grid_ptr.hh"

#include "FN_field.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Value or Field Class
 *
 * Utility class that wraps a single value and a field, to simplify accessing both of the types.
 * \{ */

template<typename T> struct ValueOrField {
  /** Value that is used when the field is empty. */
  T value{};
  fn::Field<T> field;
  VolumeGridPtr<T> grid;

  ValueOrField() = default;

  ValueOrField(T value) : value(std::move(value)) {}

  ValueOrField(fn::Field<T> field) : field(std::move(field)) {}

  ValueOrField(VolumeGridPtr<T> grid) : grid(std::move(grid)) {}

  bool is_field() const
  {
    return bool(this->field);
  }

  bool is_grid() const
  {
    return bool(this->grid);
  }

  fn::Field<T> as_field() const
  {
    if (this->field) {
      return this->field;
    }
    return fn::make_constant_field(this->value);
  }

  VolumeGridPtr<T> as_grid() const
  {
    if (this->grid) {
      return this->grid;
    }
    return grid_utils::make_empty_grid(this->value);
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
    if (value_or_field.field || value_or_field.grid) {
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
