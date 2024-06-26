/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

#include "BLI_math_vector_types.hh"

#include <cstdint>

#pragma once

/** \file
 * \ingroup bgrease_pencil
 */

struct ARegion;
struct View3D;
struct bContext;
struct Scene;
struct Object;
struct ReportList;
struct Depsgraph;

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

struct IOContext {
  ReportList *reports;
  bContext &C;
  const ARegion *region;
  const View3D *v3d;
  Scene *scene;
  Depsgraph *depsgraph;

  IOContext(bContext &C, const ARegion *region, const View3D *v3d, ReportList *reports);
};

class GreasePencilImporter {
 protected:
  const IOContext context_;
  float scale_ = 1.0f;
  int frame_number_ = 1;
  int resolution_ = 10;
  bool use_scene_unit_ = false;
  bool recenter_bounds_ = false;
  bool convert_to_poly_curves_ = false;

  Object *object_ = nullptr;

 public:
  GreasePencilImporter(const IOContext &params,
                       float scale,
                       int frame_number,
                       int resolution,
                       bool use_scene_unit,
                       bool recenter_bounds,
                       bool convert_to_poly_curves);

  Object *create_object(StringRefNull name);
  int32_t create_material(StringRefNull name, bool stroke, bool fill);
};

class GreasePencilExporter {
 protected:
  const IOContext context_;
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

 public:
  GreasePencilExporter(const IOContext &params);
};

class SVGImporter : public GreasePencilImporter {
 public:
  using GreasePencilImporter::GreasePencilImporter;

  bool read(StringRefNull filepath);
};

class SVGExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  bool write(StringRefNull filepath);
};

class PDFExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  bool write(StringRefNull filepath);
};

}  // namespace blender::io::grease_pencil
