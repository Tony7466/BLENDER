/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_report.hh"

#include "BLI_fileops.hh"

#include "IO_csv.hh"

#include "csv_data.hh"
#include "csv_string_utils.hh"

#include "csv_reader.hh"

namespace blender::io::csv {
static Vector<std::string> get_columns(const StringRef &line)
{
  Vector<std::string> columns;
  const char *p = line.begin(), *end = line.end();
  const char *cell_start = p, *cell_end = p;

  while (p != end) {
    while (*p != ',' && p != end) {
      p++;
    }
    cell_end = p;

    columns.append(std::string(cell_start, cell_end));

    if (p == end) {
      break;
    }
    p++;
    cell_start = p;
  }
  return columns;
}

static CsvColumnType get_column_type(const char *start, const char *end)
{
  bool success = false;

  int _val_int = 0;
  parse_int(start, end, success, _val_int);

  if (success) {
    return CsvColumnType::INT;
  }

  float _val_float = 0.0f;
  parse_float(start, end, success, _val_float);

  if (success) {
    return CsvColumnType::FLOAT;
  }

  // TODO: error - unsupported type
}

static Vector<CsvColumnType> get_column_types(const StringRef &line)
{
  Vector<CsvColumnType> column_types;
  const char *p = line.begin(), *end = line.end();
  const char *cell_start = p, *cell_end = p;

  while (p != end) {
    while (*p != ',' && p != end) {
      p++;
    }
    cell_end = p;

    column_types.append(get_column_type(cell_start, cell_end));

    if (p == end) {
      break;
    }
    p++;
    cell_start = p;
  }
  return column_types;
}

static int64_t get_row_count(StringRef &buffer)
{
  int64_t row_count = 1;

  while (!buffer.is_empty()) {
    read_next_line(buffer);
    row_count++;
  }

  return row_count;
}

static void parse_csv_cell(
    CsvData &csv_data, int64_t row_index, int64_t col_index, const char *start, const char *end)
{
  bool success = false;

  switch (csv_data.get_column_type(col_index)) {
    case CsvColumnType::INT: {
      int value = 0;
      parse_int(start, end, success, value);
      if (success) {
        csv_data.set_data(row_index, col_index, value);
      }  // TODO : Handle invalid value
      break;
    }
    case CsvColumnType::FLOAT: {
      float value = 0.0f;
      parse_float(start, end, success, value);
      if (success) {
        csv_data.set_data(row_index, col_index, value);
      }  // TODO : Handle invalid value
      break;
    }
  }
}

static void parse_csv_line(CsvData &csv_data, int64_t row_index, const StringRef &line)
{
  const char *p = line.begin(), *end = line.end();
  const char *cell_start = p, *cell_end = p;

  int64_t col_index = 0;

  while (p != end) {
    while (*p != ',' && p != end) {
      p++;
    }
    cell_end = p;

    parse_csv_cell(csv_data, row_index, col_index, cell_start, cell_end);

    if (p == end) {
      break;
    }
    p++;
    cell_start = p;
    col_index++;
  }
}

static void parse_csv_data(CsvData &csv_data, StringRef &buffer)
{
  int64_t row_index = 0;
  while (!buffer.is_empty()) {
    const StringRef line = read_next_line(buffer);
    parse_csv_line(csv_data, row_index, line);
    row_index++;
  }
}

PointCloud *read_csv_file(const CSVImportParams &import_params)
{
  size_t buffer_len;
  void *buffer = BLI_file_read_text_as_mem(import_params.filepath, 0, &buffer_len);

  if (buffer == nullptr) {
    fprintf(stderr, "Failed to open CSV file:'%s'.\n", import_params.filepath);
    BKE_reportf(import_params.reports,
                RPT_ERROR,
                "CSV Import: Cannot open file '%s'",
                import_params.filepath);
    return nullptr;
  }

  BLI_SCOPED_DEFER([&]() { MEM_freeN(buffer); });

  StringRef buffer_str{(const char *)buffer, int64_t(buffer_len)};

  // get row count and columns
  if (buffer_str.is_empty()) {
    fprintf(stderr, "CSV file:'%s', Is empty.\n", import_params.filepath);
    BKE_reportf(
        import_params.reports, RPT_ERROR, "CSV Import: empty file '%s'", import_params.filepath);
    return nullptr;
  }

  const StringRef header = read_next_line(buffer_str);
  const Vector<std::string> columns = get_columns(header);

  if (buffer_str.is_empty()) {
    fprintf(stderr, "CSV file:'%s', Has no points.\n", import_params.filepath);
    BKE_reportf(
        import_params.reports, RPT_ERROR, "CSV Import: no points '%s'", import_params.filepath);
    return nullptr;
  }

  // shallow copy buffer to preserve pointers from first row for parsing
  StringRef data_buffer(buffer_str.begin(), buffer_str.end());

  const StringRef first_row = read_next_line(buffer_str);
  const Vector<CsvColumnType> column_types = get_column_types(first_row);

  const int64_t row_count = get_row_count(buffer_str);

  // create csv data
  CsvData csv_data(row_count, columns, column_types);

  // fill csv data while seeking over the file
  parse_csv_data(csv_data, data_buffer);

  // return point cloud from csv data
  return csv_data.to_point_cloud();
}
}  // namespace blender::io::csv
