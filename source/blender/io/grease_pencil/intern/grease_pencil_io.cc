/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_math_vector.h"

#include "BKE_context.hh"
#include "BKE_gpencil_legacy.h"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "ED_object.hh"

#include "grease_pencil_io.hh"

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

IOContext::IOContext(bContext &C, const ARegion *region, const View3D *v3d, ReportList *reports)
    : reports(reports),
      C(C),
      region(region),
      v3d(v3d),
      scene(CTX_data_scene(&C)),
      depsgraph(CTX_data_depsgraph_pointer(&C))
{
}

GreasePencilImporter::GreasePencilImporter(const IOContext &params,
                                           const float scale,
                                           const int frame_number,
                                           int resolution,
                                           const bool use_scene_unit,
                                           const bool recenter_bounds,
                                           const bool convert_to_poly_curves)
    : context_(params),
      scale_(scale),
      frame_number_(frame_number),
      resolution_(resolution),
      use_scene_unit_(use_scene_unit),
      recenter_bounds_(recenter_bounds),
      convert_to_poly_curves_(convert_to_poly_curves)
{
}

GreasePencilExporter::GreasePencilExporter(const IOContext &params) : context_(params) {}

Object *GreasePencilImporter::create_object(const StringRefNull name)
{
  const float3 cur_loc = context_.scene->cursor.location;
  const float3 rot = float3(0.0f);
  const ushort local_view_bits = (context_.v3d && context_.v3d->localvd) ?
                                     context_.v3d->local_view_uid :
                                     ushort(0);

  Object *ob_gpencil = blender::ed::object::add_type(
      &context_.C, OB_GREASE_PENCIL, name.c_str(), cur_loc, rot, false, local_view_bits);

  return ob_gpencil;
}

int32_t GreasePencilImporter::create_material(const StringRefNull name,
                                              const bool stroke,
                                              const bool fill)
{
  const ColorGeometry4f default_stroke_color = {0.0f, 0.0f, 0.0f, 1.0f};
  const ColorGeometry4f default_fill_color = {0.5f, 0.5f, 0.5f, 1.0f};
  int32_t mat_index = BKE_gpencil_object_material_index_get_by_name(object_, name.c_str());
  /* Stroke and Fill material. */
  if (mat_index == -1) {
    Main *bmain = CTX_data_main(&context_.C);
    int32_t new_idx;
    Material *mat_gp = BKE_gpencil_object_material_new(bmain, object_, name.c_str(), &new_idx);
    MaterialGPencilStyle *gp_style = mat_gp->gp_style;
    gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
    gp_style->flag &= ~GP_MATERIAL_FILL_SHOW;

    copy_v4_v4(gp_style->stroke_rgba, default_stroke_color);
    copy_v4_v4(gp_style->fill_rgba, default_fill_color);
    if (stroke) {
      gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
    }
    if (fill) {
      gp_style->flag |= GP_MATERIAL_FILL_SHOW;
    }
    mat_index = object_->totcol - 1;
  }

  return mat_index;
}

}  // namespace blender::io::grease_pencil
