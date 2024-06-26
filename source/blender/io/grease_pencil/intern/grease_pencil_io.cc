/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.h"

#include "BKE_camera.h"
#include "BKE_context.hh"
#include "BKE_gpencil_legacy.h"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_scene.hh"

#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"

#include "ED_object.hh"
#include "ED_view3d.hh"

#include "grease_pencil_io.hh"

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

IOContext::IOContext(bContext &C,
                     const ARegion *region,
                     const View3D *v3d,
                     const RegionView3D *rv3d,
                     ReportList *reports)
    : reports(reports),
      C(C),
      region(region),
      v3d(v3d),
      rv3d(rv3d),
      scene(CTX_data_scene(&C)),
      depsgraph(CTX_data_depsgraph_pointer(&C))
{
}

GreasePencilImporter::GreasePencilImporter(const IOContext &context, const ImportParams &params)
    : context_(context), params_(params)
{
}

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

GreasePencilExporter::GreasePencilExporter(const IOContext &context, const ExportParams &params)
    : context_(context), params_(params)
{
}

void GreasePencilExporter::prepare_camera_params(Scene &scene, const bool force_camera_view)
{
  const bool use_camera_view = force_camera_view && (context_.v3d->camera != nullptr);

  /* Ensure camera switch is applied. */
  BKE_scene_camera_switch_update(&scene);

  /* Calculate camera matrix. */
  Object *cam_ob = scene.camera;
  if (cam_ob != nullptr) {
    /* Set up parameters. */
    CameraParams params;
    BKE_camera_params_init(&params);
    BKE_camera_params_from_object(&params, cam_ob);

    /* Compute matrix, view-plane, etc. */
    BKE_camera_params_compute_viewplane(
        &params, scene.r.xsch, scene.r.ysch, scene.r.xasp, scene.r.yasp);
    BKE_camera_params_compute_matrix(&params);

    float4x4 viewmat = math::invert(cam_ob->object_to_world());
    persmat_ = float4x4(params.winmat) * viewmat;
  }
  else {
    persmat_ = float4x4::identity();
  }

  win_size_ = {context_.region->winx, context_.region->winy};

  /* Camera rectangle. */
  if ((context_.rv3d->persp == RV3D_CAMOB) || (use_camera_view)) {
    BKE_render_resolution(&scene.r, false, &render_size_.x, &render_size_.y);

    ED_view3d_calc_camera_border(&scene,
                                 context_.depsgraph,
                                 context_.region,
                                 context_.v3d,
                                 context_.rv3d,
                                 &camera_rect_,
                                 true);
    is_camera_ = true;
    camera_ratio_ = render_size_.x / (camera_rect_.xmax - camera_rect_.xmin);
    offset_.x = camera_rect_.xmin;
    offset_.y = camera_rect_.ymin;
  }
  else {
    is_camera_ = false;
    /* Calc selected object boundbox. Need set initial value to some variables. */
    camera_ratio_ = 1.0f;
    offset_.x = 0.0f;
    offset_.y = 0.0f;

    // create_object_list();

    // selected_objects_boundbox_calc();
    // rctf boundbox;
    // selected_objects_boundbox_get(&boundbox);

    // render_x_ = boundbox.xmax - boundbox.xmin;
    // render_y_ = boundbox.ymax - boundbox.ymin;
    // offset_.x = boundbox.xmin;
    // offset_.y = boundbox.ymin;
  }
}

}  // namespace blender::io::grease_pencil
