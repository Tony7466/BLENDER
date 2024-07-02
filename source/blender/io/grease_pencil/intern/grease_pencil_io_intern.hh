/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_function_ref.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_vec_types.h"

#include "grease_pencil_io.hh"

#include <cstdint>

#pragma once

/** \file
 * \ingroup bgrease_pencil
 */

struct Scene;
struct Object;
struct GreasePencil;
namespace blender::bke::greasepencil {
class Layer;
}  // namespace blender::bke::greasepencil

namespace blender::io::grease_pencil {

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
 public:
  struct ObjectInfo {
    const Object *object;
    float depth;
  };

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

  Vector<ObjectInfo> retrieve_objects() const;
};

}  // namespace blender::io::grease_pencil
