/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix_types.hh"
#include "BLI_string_ref.hh"

#include "DNA_vec_types.h"
#include "DNA_view3d_types.h"

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

// /* GpencilIOParams->flag. */
// enum class IOParamsFlag {
//   /* Export Filled strokes. */
//   ExportFill = (1 << 0),
//   /* Export normalized thickness. */
//   ExportNormalizedThickness = (1 << 1),
//   /* Clip camera area. */
//   ExportClipCamera = (1 << 2),
// };
// ENUM_OPERATORS(IOParamsFlag, IOParamsFlag::ExportClipCamera);

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
  const RegionView3D *rv3d;
  Scene *scene;
  Depsgraph *depsgraph;

  IOContext(bContext &C,
            const ARegion *region,
            const View3D *v3d,
            const RegionView3D *rv3d,
            ReportList *reports);
};

struct ImportParams {
  float scale = 1.0f;
  int frame_number = 1;
  int resolution = 10;
  bool use_scene_unit = false;
  bool recenter_bounds = false;
  bool convert_to_poly_curves = false;
};

struct ExportParams {};

class GreasePencilImporter {
 protected:
  const IOContext context_;
  const ImportParams params_;

  Object *object_ = nullptr;

 public:
  GreasePencilImporter(const IOContext &context, const ImportParams &params);

  Object *create_object(StringRefNull name);
  int32_t create_material(StringRefNull name, bool stroke, bool fill);
};

class GreasePencilExporter {
 protected:
  const IOContext context_;
  const ExportParams params_;

  /* Camera parameters. */
  float4x4 persmat_;
  int2 win_size_;
  int2 render_size_;
  bool is_camera_;
  float camera_ratio_;
  rctf camera_rect_;

  float2 offset_;

 public:
  GreasePencilExporter(const IOContext &context, const ExportParams &params);

  // XXX force_camera_view should be true for PDF export
  void prepare_camera_params(Scene &scene, bool force_camera_view);
};

bool import_svg(const IOContext &context, const ImportParams &params, StringRefNull filepath);
bool export_svg(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath);
bool export_pdf(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath);

}  // namespace blender::io::grease_pencil
