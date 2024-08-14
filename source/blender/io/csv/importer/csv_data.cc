/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "csv_data.hh"

namespace blender::io::csv {
void CsvData::add_column(std::string &name, CsvColumnType &type)
{
  CsvColumn column{};

  column.name = name;
  column.type = type;
  column.vector = create_vector_for_type(type);

  this->data.add(name, column);
}

void CsvData::add_data_to_column(std::string &name, void *data)
{
  CsvColumn column = this->data.lookup(name);

  switch (column.type) {
    case CsvColumnType::INT: {
      blender::Vector<int> *vector = reinterpret_cast<blender::Vector<int> *>(column.vector);
      vector->append(*reinterpret_cast<int *>(data));
      break;
    }
    case CsvColumnType::FLOAT: {
      blender::Vector<float> *vector = reinterpret_cast<blender::Vector<float> *>(column.vector);
      vector->append(*reinterpret_cast<float *>(data));
      break;
    }
    case CsvColumnType::STRING: {
      blender::Vector<std::string> *vector = reinterpret_cast<blender::Vector<std::string> *>(
          column.vector);
      vector->append(*reinterpret_cast<std::string *>(data));
      break;
    }
  }
}

void *CsvData::create_vector_for_type(CsvColumnType &type)
{
  switch (type) {
    case CsvColumnType::INT: {
    }
    case CsvColumnType::FLOAT: {
    }
  }
}
}  // namespace blender::io::csv
