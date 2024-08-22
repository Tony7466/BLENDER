/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#pragma once

#include "IO_csv.hh"

struct PointCloud;

namespace blender::io::csv {
PointCloud *importer_point_cloud(const CSVImportParams &import_params);
}
