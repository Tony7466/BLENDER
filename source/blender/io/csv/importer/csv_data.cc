/* SPDX-FileCopyrightText: 2023 Blender Authors
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
CsvData::CsvData(int64_t row_count,
                 Vector<std::string> column_names,
                 Vector<eCustomDataType> column_types)
    : data(column_names.size()),
      row_count(row_count),
      column_count(column_names.size()),
      column_names(column_names.as_span()),
      column_types(column_types.as_span())
{
  for (int i = 0; i < this->column_count; i++) {
    data[i] = create_garray_for_type(this->column_types[i]);
  }
}

PointCloud *CsvData::to_point_cloud() const
{
  PointCloud *point_cloud = BKE_pointcloud_new_nomain(row_count);

  // set all positions to be zero
  point_cloud->positions_for_write().fill(float3(0.0f, 0.0f, 0.0f));

  // fill the attributes
  for (int i = 0; i < column_count; i++) {
    const std::string column_name = column_names[i];
    const eCustomDataType column_type = column_types[i];
    const void *column_data = data[i].data();
    CustomData_add_layer_named_with_data(&point_cloud->pdata,
                                         column_type,
                                         MEM_dupallocN(column_data),
                                         row_count,
                                         column_name,
                                         nullptr);
  }

  return point_cloud;
}

GArray<> CsvData::create_garray_for_type(eCustomDataType &type)
{
  switch (type) {
    case eCustomDataType::CD_PROP_INT32: {
      return GArray(CPPType::get<int>(), row_count);
    }
    case eCustomDataType::CD_PROP_FLOAT: {
      return GArray(CPPType::get<float>(), row_count);
    }
    default: {
      // This will never happen
      return GArray<>();
    }
  }
}
}  // namespace blender::io::csv
