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
  blender::Map<std::string, GArray<>> data;

  int64_t row_count;
  int64_t column_count;

 public:
  CsvData(int64_t row_count, Array<std::pair<std::string, CsvColumnType>> columns);

  template<typename T> void set_data(int64_t row_index, std::string &name, T value);

  PointCloud *to_point_cloud() const;

 private:
  GArray<> create_garray_for_type(CsvColumnType &type);
};
}  // namespace blender::io::csv
