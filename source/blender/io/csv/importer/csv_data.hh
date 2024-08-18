/* SPDX-FileCopyrightText: 2023 Blender Authors
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
enum class CsvColumnType { INT, FLOAT };

class CsvData {
 private:
  Array<GArray<>> data;

  int64_t row_count;
  int64_t column_count;

  Array<std::string> column_names;
  Array<CsvColumnType> column_types;

 public:
  CsvData(int64_t row_count, Vector<std::string> column_names, Vector<CsvColumnType> column_types);

  template<typename T> void set_data(int64_t row_index, int64_t col_index, T value);

  PointCloud *to_point_cloud() const;

  inline CsvColumnType get_column_type(int64_t col_index) const
  {
    return column_types[col_index];
  }

  inline std::string get_column_name(int64_t col_index) const
  {
    return column_names[col_index];
  }

 private:
  GArray<> create_garray_for_type(CsvColumnType &type);
};
}  // namespace blender::io::csv
