/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BKE_report.hh"

#include "BLI_fileops.hh"

#include "csv_reader.hh"

#include "csv_import.hh"

namespace blender::io::csv {

PointCloud *importer_point_cloud(const CSVImportParams &import_params)
{
  return read_csv_file(import_params);
}

}  // namespace blender::io::csv
