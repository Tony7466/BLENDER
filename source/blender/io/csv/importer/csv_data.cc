/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_pointcloud.hh"

#include "csv_data.hh"

namespace blender::io::csv {
CsvData::CsvData(int64_t row_count, Array<std::pair<std::string, CsvColumnType>> columns)
{
  this->row_count = row_count;
  this->column_count = columns.size();

  for (std::pair<std::string, CsvColumnType> item : columns) {
    data.add(item.first, create_garray_for_type(item.second));
  }
}

template<typename T> void CsvData::set_data(int64_t row_index, std::string &name, T value)
{
  data.lookup(name)[row_index] = value;
}

PointCloud *CsvData::to_point_cloud() const
{
  return nullptr;
}

GArray<> CsvData::create_garray_for_type(CsvColumnType &type)
{
  switch (type) {
    case CsvColumnType::INT: {
      return GArray(CPPType::get<int>(), row_count);
    }
    case CsvColumnType::FLOAT: {
      return GArray(CPPType::get<float>(), row_count);
    }
  }
}
}  // namespace blender::io::csv
