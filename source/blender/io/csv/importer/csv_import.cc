/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_report.hh"

#include "BLI_fileops.hh"

#include "csv_import.hh"

namespace blender::io::csv {
PointCloud *importer_point_cloud(const CSVImportParams &import_params)
{
  FILE *file = BLI_fopen(import_params.filepath, "rb");
  if (!file) {
    fprintf(stderr, "Failed to open CSV file:'%s'.\n", import_params.filepath);
    BKE_reportf(import_params.reports,
                RPT_ERROR,
                "CSV Import: Cannot open file '%s'",
                import_params.filepath);
    return nullptr;
  }
  BLI_SCOPED_DEFER([&]() { fclose(file); });
}
}  // namespace blender::io::csv
