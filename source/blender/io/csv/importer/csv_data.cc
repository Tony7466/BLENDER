/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_pointcloud.hh"

#include "csv_data.hh"

namespace blender::io::csv {
CsvData::CsvData(int64_t row_count,
                 Vector<std::string> column_names,
                 Vector<CsvColumnType> column_types)
    : row_count(row_count),
      column_count(column_names.size()),
      column_names(column_names.as_span()),
      column_types(column_types.as_span())
{
  for (int i = 0; i < this->column_count; i++) {
    data[i] = create_garray_for_type(this->column_types[i]);
  }
}

// template<typename T> void CsvData::set_data(int64_t row_index, int64_t col_index, T value)
// {
//   GMutableSpan mutable_span = data[col_index].as_mutable_span();
//   MutableSpan typed_mutable_span = mutable_span.typed<T>();
//   typed_mutable_span[row_index] = value;
// }

// void CsvData::set_data_int(int64_t row_index, int64_t col_index, int value)
// {
//   GMutableSpan mutable_span = data[col_index].as_mutable_span();
//   MutableSpan typed_mutable_span = mutable_span.typed<int>();
//   typed_mutable_span[row_index] = value;
// }

// void CsvData::set_data_float(int64_t row_index, int64_t col_index, float value)
// {
//   GMutableSpan mutable_span = data[col_index].as_mutable_span();
//   MutableSpan typed_mutable_span = mutable_span.typed<float>();
//   typed_mutable_span[row_index] = value;
// }

PointCloud *CsvData::to_point_cloud() const
{
  PointCloud *point_cloud = BKE_pointcloud_new_nomain(row_count);
  // fill the attributes
  return point_cloud;
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
