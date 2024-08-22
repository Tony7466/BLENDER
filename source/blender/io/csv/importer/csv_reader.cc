/* SPDX-FileCopyrightText: 2024 Blender Authors
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

static Vector<std::string> get_columns(const StringRef line)
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

static bool get_column_type(const char *start, const char *end, eCustomDataType &column_type)
{
  bool success = false;

  int _val_int = 0;
  parse_int(start, end, success, _val_int);

  if (success) {
    column_type = eCustomDataType::CD_PROP_INT32;
    return true;
  }

  float _val_float = 0.0f;
  parse_float(start, end, success, _val_float);

  if (success) {
    column_type = eCustomDataType::CD_PROP_FLOAT;
    return true;
  }

  return false;
}

static bool get_column_types(const StringRef line, Vector<eCustomDataType> &column_types)
{
  const char *p = line.begin(), *end = line.end();
  const char *cell_start = p, *cell_end = p;

  while (p != end) {
    while (*p != ',' && p != end) {
      p++;
    }
    cell_end = p;

    eCustomDataType column_type;
    if (!get_column_type(cell_start, cell_end, column_type)) {
      return false;
    }
    column_types.append(column_type);

    if (p == end) {
      break;
    }
    p++;
    cell_start = p;
  }

  return true;
}

static int64_t get_row_count(StringRef buffer)
{
  int64_t row_count = 1;

  while (!buffer.is_empty()) {
    read_next_line(buffer);
    row_count++;
  }

  return row_count;
}

static bool parse_csv_cell(CsvData &csv_data,
                           int64_t row_index,
                           int64_t col_index,
                           const char *start,
                           const char *end,
                           const CSVImportParams &import_params)
{
  bool success = false;

  switch (csv_data.get_column_type(col_index)) {
    case CD_PROP_INT32: {
      int value = 0;
      parse_int(start, end, success, value);
      if (success) {
        csv_data.set_data(row_index, col_index, value);
      }
      else {
        std::string column_name = csv_data.get_column_name(col_index);
        fprintf(
            stderr,
            "CSV file: '%s', unexpected value found at row %lld for column %s of type Integer.\n",
            import_params.filepath,
            row_index,
            column_name.c_str());
        BKE_reportf(import_params.reports,
                    RPT_ERROR,
                    "CSV Import: file '%s' has an unexpected value at row %lld for column %s of "
                    "type Integer",
                    import_params.filepath,
                    row_index,
                    column_name.c_str());
        return false;
      }
      break;
    }
    case CD_PROP_FLOAT: {
      float value = 0.0f;
      parse_float(start, end, success, value);
      if (success) {
        csv_data.set_data(row_index, col_index, value);
      }
      else {
        std::string column_name = csv_data.get_column_name(col_index);
        fprintf(
            stderr,
            "CSV file: '%s', unexpected value found at row %lld for column %s of type Float.\n",
            import_params.filepath,
            row_index,
            column_name.c_str());
        BKE_reportf(import_params.reports,
                    RPT_ERROR,
                    "CSV Import: file '%s' has an unexpected value at row %lld for column %s of "
                    "type Float",
                    import_params.filepath,
                    row_index,
                    column_name.c_str());
        return false;
      }
      break;
    }
    default: {
      return false;
    }
  }

  return true;
}

static bool parse_csv_line(CsvData &csv_data,
                           int64_t row_index,
                           const StringRef line,
                           const CSVImportParams &import_params)
{
  const char *p = line.begin(), *end = line.end();
  const char *cell_start = p, *cell_end = p;

  int64_t col_index = 0;

  while (p != end) {
    while (*p != ',' && p != end) {
      p++;
    }
    cell_end = p;

    if (!parse_csv_cell(csv_data, row_index, col_index, cell_start, cell_end, import_params)) {
      return false;
    }
    col_index++;

    if (p == end) {
      break;
    }
    p++;
    cell_start = p;
  }

  return true;
}

static bool parse_csv_data(CsvData &csv_data,
                           StringRef buffer,
                           const CSVImportParams &import_params)
{
  int64_t row_index = 0;
  while (!buffer.is_empty()) {
    const StringRef line = read_next_line(buffer);

    if (!parse_csv_line(csv_data, row_index, line, import_params)) {
      return false;
    }

    row_index++;
  }

  return true;
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
    fprintf(stderr, "CSV file: '%s', Is empty.\n", import_params.filepath);
    BKE_reportf(
        import_params.reports, RPT_ERROR, "CSV Import: empty file '%s'", import_params.filepath);
    return nullptr;
  }

  const StringRef header = read_next_line(buffer_str);
  const Vector<std::string> columns = get_columns(header);

  if (buffer_str.is_empty()) {
    fprintf(stderr, "CSV file: '%s', Has no rows.\n", import_params.filepath);
    BKE_reportf(import_params.reports,
                RPT_ERROR,
                "CSV Import: no rows in file '%s'",
                import_params.filepath);
    return nullptr;
  }

  // shallow copy buffer to preserve pointers from first row for parsing
  StringRef data_buffer(buffer_str.begin(), buffer_str.end());

  const StringRef first_row = read_next_line(buffer_str);

  Vector<eCustomDataType> column_types;
  if (!get_column_types(first_row, column_types)) {
    std::string column_name = columns[column_types.size()];
    fprintf(stderr,
            "CSV file: '%s', Column %s is of unsupported data type.\n",
            import_params.filepath,
            column_name.c_str());
    BKE_reportf(import_params.reports,
                RPT_ERROR,
                "CSV Import: file '%s', Column %s is of unsupported data type",
                import_params.filepath,
                column_name.c_str());
    return nullptr;
  }

  const int64_t row_count = get_row_count(buffer_str);

  // create csv data
  CsvData csv_data(row_count, columns, column_types);

  // fill csv data while seeking over the file
  if (parse_csv_data(csv_data, data_buffer, import_params)) {
    // return point cloud from csv data
    return csv_data.to_point_cloud();
  }

  return nullptr;
}

}  // namespace blender::io::csv
