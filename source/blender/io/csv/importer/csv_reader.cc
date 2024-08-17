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
static Vector<std::string> get_columns(const StringRef &line) {}

static Vector<CsvColumnType> get_column_types(const StringRef &line) {}

static int64_t get_row_count(StringRef &buffer) {}

static void parse_csv_data(CsvData &csv_data, StringRef &buffer) {}

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
