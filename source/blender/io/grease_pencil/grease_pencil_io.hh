/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

#include <cstdint>

#pragma once

/** \file
 * \ingroup bgrease_pencil
 */

struct ARegion;
struct Object;
struct View3D;
struct bContext;

namespace blender::io::grease_pencil {

/* GpencilIOParams->flag. */
enum class IOParamsFlag {
  /* Export Filled strokes. */
  ExportFill = (1 << 0),
  /* Export normalized thickness. */
  ExportNormalizedThickness = (1 << 1),
  /* Clip camera area. */
  ExportClipCamera = (1 << 2),
};
ENUM_OPERATORS(IOParamsFlag, IOParamsFlag::ExportClipCamera);

/* Object to be exported. */
enum class ExportSelect {
  Active = 0,
  Selected = 1,
  Visible = 2,
};

/** Frame-range to be exported. */
enum ExportFrame {
  ActiveFrame = 0,
  SelectedFrame = 1,
  SceneFrame = 2,
};

struct IOParams {
  bContext *C;
  ARegion *region;
  View3D *v3d;
  /** Grease pencil object. */
  Object *ob;
  int32_t frame_start;
  int32_t frame_end;
  int32_t frame_cur;
  /* #IOParamsFlag. */
  int flag;
  float scale;
  ExportSelect select_mode;
  ExportFrame frame_mode;
  /** Stroke sampling factor. */
  float stroke_sample;
  int32_t resolution;
  /** Filename to be used in new objects. */
  StringRef filename;
};

bool grease_pencil_io_import_svg(StringRef filepath, struct IOParams &params);
bool grease_pencil_io_export_svg(StringRef filepath, struct IOParams &params);
bool grease_pencil_io_export_pdf(StringRef filepath, struct IOParams &params);

}  // namespace blender::io::grease_pencil
