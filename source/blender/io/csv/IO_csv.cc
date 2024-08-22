/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "BLI_timeit.hh"

#include "IO_csv.hh"

#include "csv_import.hh"

PointCloud *CSV_import_point_cloud(const CSVImportParams *import_params)
{
  return blender::io::csv::importer_point_cloud(*import_params);
}
