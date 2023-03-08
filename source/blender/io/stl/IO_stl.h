/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#pragma once

#include "BKE_context.h"
#include "BLI_path_util.h"
#include "IO_orientation.h"

#ifdef __cplusplus
extern "C" {
#endif

struct STLImportParams {
  /** Full path to the source STL file to import. */
  char filepath[FILE_MAX];
  eIOAxis forward_axis;
  eIOAxis up_axis;
  bool use_facet_normal;
  bool use_scene_unit;
  float global_scale;
  bool use_mesh_validate;
};

struct STLExportParams {
  /** Full path to the to-be-saved STL file. */
  char filepath[FILE_MAX];
  eIOAxis forward_axis;
  eIOAxis up_axis;
  bool use_selection_only;
  bool use_scene_unit;
  bool use_apply_modifiers;
  bool use_ascii;
  bool use_batch;
  float global_scale;
};

/**
 * C-interface for the importer.
 */
void STL_import(bContext *C, const struct STLImportParams *import_params);

/**
 * C-interface for the exporter.
 */
void STL_export(bContext *C, const struct STLExportParams *export_params);

#ifdef __cplusplus
}
#endif
