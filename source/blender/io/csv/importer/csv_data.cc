/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_attribute.hh"
#include "BKE_customdata.hh"
#include "BKE_pointcloud.hh"

#include "csv_data.hh"

namespace blender::io::csv {

CsvData::CsvData(const int64_t rows_num,
                 const Span<std::string> column_names,
                 const Span<eCustomDataType> column_types)
    : data(column_names.size()),
      rows_num(rows_num),
      columns_num(column_names.size()),
      column_names(column_names),
      column_types(column_types)
{
  for (int i = 0; i < this->columns_num; i++) {
    data[i] = create_garray_for_type(this->column_types[i]);
  }
}

PointCloud *CsvData::to_point_cloud() const
{
  PointCloud *point_cloud = BKE_pointcloud_new_nomain(rows_num);

  // set all positions to be zero
  point_cloud->positions_for_write().fill(float3(0.0f, 0.0f, 0.0f));

  // fill the attributes
  for (int i = 0; i < columns_num; i++) {
    const std::string column_name = column_names[i];
    const eCustomDataType column_type = column_types[i];
    void *column_data = get_data_of_garray(data[i], column_type);
    CustomData_add_layer_named_with_data(
        &point_cloud->pdata, column_type, column_data, rows_num, column_name, nullptr);
  }

  return point_cloud;
}

GArray<> CsvData::create_garray_for_type(const eCustomDataType type) const
{
  switch (type) {
    case CD_PROP_INT32: {
      return GArray(CPPType::get<int>(), rows_num);
    }
    case CD_PROP_FLOAT: {
      return GArray(CPPType::get<float>(), rows_num);
    }
    default: {
      BLI_assert_unreachable();
      return GArray<>();
    }
  }
}

void *CsvData::get_data_of_garray(const GArray<> array, const eCustomDataType type) const
{
  switch (type) {
    case CD_PROP_INT32: {
      Array<int> *data = new Array<int>(array.as_span().typed<int>());
      return data->data();
    }
    case CD_PROP_FLOAT: {
      Array<float> *data = new Array<float>(array.as_span().typed<float>());
      return data->data();
    }
    default: {
      BLI_assert_unreachable();
      return nullptr;
    }
  }
}

}  // namespace blender::io::csv
