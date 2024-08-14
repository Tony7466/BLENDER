/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#pragma once

#include <string>

#include "BLI_map.hh"
#include "BLI_vector.hh"

namespace blender::io::csv {
enum class CsvColumnType { INT, FLOAT, STRING };

struct CsvColumn {
  std::string name;
  CsvColumnType type;
  void *vector;
};

class CsvData {
 private:
  blender::Map<std::string, CsvColumn> data;

 public:
  // CsvData();

  void add_column(std::string &name, CsvColumnType &type);
  void add_data_to_column(std::string &name, void *data);

 private:
  void *create_vector_for_type(CsvColumnType &type);
};
}  // namespace blender::io::csv
