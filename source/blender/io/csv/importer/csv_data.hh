/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#pragma once

#include <string>

#include "BLI_array.hh"
#include "BLI_generic_array.hh"
#include "BLI_map.hh"

struct PointCloud;

namespace blender::io::csv {

class CsvData {
 private:
  Array<GArray<>> data;

  int64_t rows_num;
  int64_t columns_num;

  Array<std::string> column_names;
  Array<eCustomDataType> column_types;

 public:
  CsvData(const int64_t rows_num,
          const Span<std::string> column_names,
          const Span<eCustomDataType> column_types);

  PointCloud *to_point_cloud() const;

  template<typename T>
  void set_data(const int64_t row_index, const int64_t col_index, const T value)
  {
    GMutableSpan mutable_span = data[col_index].as_mutable_span();
    MutableSpan typed_mutable_span = mutable_span.typed<T>();
    typed_mutable_span[row_index] = value;
  }

  eCustomDataType get_column_type(const int64_t col_index) const
  {
    return column_types[col_index];
  }

  StringRef get_column_name(const int64_t col_index) const
  {
    return column_names[col_index];
  }

 private:
  GArray<> create_garray_for_type(const eCustomDataType type) const;
  void *get_data_of_garray(const GArray<> array, const eCustomDataType type) const;
};

}  // namespace blender::io::csv
