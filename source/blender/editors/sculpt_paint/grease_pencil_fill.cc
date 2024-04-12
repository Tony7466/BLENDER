/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_rect.h"
#include "BLI_task.hh"

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_image.h"
#include "BKE_lib_id.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph_query.hh"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DNA_view3d_types.h"
#include "DNA_windowmanager_types.h"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "GPU_framebuffer.hh"
#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"

namespace blender::ed::greasepencil {

/* Helper: Calc the maximum bounding box size of strokes to get the zoom level of the viewport.
 * For each stroke, the 2D projected bounding box is calculated and using this data, the total
 * object bounding box (all strokes) is calculated. */
static rctf get_boundary_bounds(const ARegion &region,
                                const Brush &brush,
                                Object &object,
                                const Object &object_eval,
                                const Span<DrawingInfo> src_drawings,
                                const VArray<bool> &is_boundary_layer)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  rctf bounds;
  BLI_rctf_init_minmax(&bounds);

  if (brush.gpencil_settings->flag & GP_BRUSH_FILL_FIT_DISABLE) {
    return bounds;
  }

  BLI_assert(object.type == OB_GREASE_PENCIL);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object.data);

  BLI_assert(grease_pencil.has_active_layer());

  for (const DrawingInfo &info : src_drawings) {
    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
            &object_eval, object, info.layer_index, info.frame_number);
    const bool only_boundary_strokes = is_boundary_layer[info.layer_index];
    const bke::CurvesGeometry &strokes = info.drawing.strokes();
    const bke::AttributeAccessor attributes = strokes.attributes();
    const VArray<int> materials = *attributes.lookup<int>("material_index",
                                                          bke::AttrDomain::Curve);
    const VArray<bool> is_boundary_stroke = *attributes.lookup_or_default<bool>(
        "is_boundary", bke::AttrDomain::Curve, false);

    threading::parallel_for(strokes.curves_range(), 512, [&](const IndexRange range) {
      for (const int curve_i : range) {
        const IndexRange points = strokes.points_by_curve()[curve_i];
        /* Check if stroke can be drawn. */
        if (points.size() < 2) {
          continue;
        }
        /* check if the color is visible */
        const int material_index = materials[curve_i];
        Material *mat = BKE_object_material_get(&object, material_index + 1);
        if (mat == 0 || (mat->gp_style->flag & GP_MATERIAL_HIDE)) {
          continue;
        }

        /* If the layer must be skipped, but the stroke is not boundary, skip stroke. */
        if (only_boundary_strokes && !is_boundary_stroke[curve_i]) {
          continue;
        }

        for (const int point_i : points) {
          float2 pos_view;
          eV3DProjStatus result = ED_view3d_project_float_global(
              &region, deformation.positions[point_i], pos_view, V3D_PROJ_TEST_NOP);
          if (result == V3D_PROJ_RET_OK) {
            BLI_rctf_do_minmax_v(&bounds, pos_view);
          }
        }
      }
    });
  }

  return bounds;
}

static rctf get_region_bounds(const ARegion &region)
{
  /* Init maximum boundbox size. */
  rctf region_bounds;
  const float winx_half = region.winx / 2.0f;
  const float winy_half = region.winy / 2.0f;
  BLI_rctf_init(&region_bounds,
                0.0f - winx_half,
                region.winx + winx_half,
                0.0f - winy_half,
                region.winy + winy_half);
  return region_bounds;
}

/* Draw strokes in off-screen buffer. */
static Image *render_offscreen(Main &bmain,
                               ARegion &region,
                               const Scene &scene,
                               Depsgraph &depsgraph,
                               View3D &view3d,
                               const RegionView3D &rv3d,
                               const float pixel_scale,
                               const float2 zoom,
                               const float2 offset,
                               ReportList &reports)
{
  const int min_window_size = 128;
  const int2 win_size = math::max(int2(region.winx, region.winy) * pixel_scale,
                                  int2(min_window_size));

  char err_out[256] = "unknown";
  GPUOffScreen *offscreen = GPU_offscreen_create(
      win_size.x, win_size.y, true, GPU_RGBA8, GPU_TEXTURE_USAGE_HOST_READ, err_out);
  if (offscreen == nullptr) {
    BKE_report(&reports, RPT_ERROR, "Unable to create fill buffer");
    return nullptr;
  }

  auto restore_region =
      [&region, winx = region.winx, winy = region.winy, winrct = region.winrct]() {
        region.winx = winx;
        region.winy = winy;
        region.winrct = winrct;
      };

  /* Resize region. */
  region.winrct.xmin = 0;
  region.winrct.ymin = 0;
  region.winrct.xmax = win_size.x;
  region.winrct.ymax = win_size.y;
  region.winx = short(win_size.x);
  region.winy = short(win_size.y);

  GPU_offscreen_bind(offscreen, true);
  const uint imb_flag = IB_rectfloat;
  ImBuf *ibuf = IMB_allocImBuf(win_size.x, win_size.y, 32, imb_flag);

  rctf viewplane;
  float clip_start, clip_end;
  const bool is_ortho = ED_view3d_viewplane_get(&depsgraph,
                                                &view3d,
                                                &rv3d,
                                                win_size.x,
                                                win_size.y,
                                                &viewplane,
                                                &clip_start,
                                                &clip_end,
                                                nullptr);

  /* Rescale `viewplane` to fit all strokes. */
  viewplane.xmin = viewplane.xmin * zoom.x + offset.x;
  viewplane.xmax = viewplane.xmax * zoom.x + offset.x;
  viewplane.ymin = viewplane.ymin * zoom.y + offset.y;
  viewplane.ymax = viewplane.ymax * zoom.y + offset.y;

  float4x4 winmat;
  if (is_ortho) {
    orthographic_m4(winmat.ptr(),
                    viewplane.xmin,
                    viewplane.xmax,
                    viewplane.ymin,
                    viewplane.ymax,
                    -clip_end,
                    clip_end);
  }
  else {
    perspective_m4(winmat.ptr(),
                   viewplane.xmin,
                   viewplane.xmax,
                   viewplane.ymin,
                   viewplane.ymax,
                   clip_start,
                   clip_end);
  }

  GPU_matrix_push_projection();
  GPU_matrix_identity_projection_set();
  GPU_matrix_push();
  GPU_matrix_identity_set();

  GPU_depth_mask(true);
  GPU_clear_color(0.0f, 0.0f, 0.0f, 0.0f);
  GPU_clear_depth(1.0f);

  ED_view3d_update_viewmat(
      &depsgraph, &scene, &view3d, &region, nullptr, winmat.ptr(), nullptr, true);
  /* Set for OpenGL. */
  GPU_matrix_projection_set(rv3d.winmat);
  GPU_matrix_set(rv3d.viewmat);

  /* draw strokes */
  const ColorGeometry4f ink(1.0f, 0.0f, 0.0f, 1.0f);
  // gpencil_draw_datablock(tgpf, ink); // <<<<<<<<<<<<<< TODO!!!

  GPU_depth_mask(false);

  GPU_matrix_pop_projection();
  GPU_matrix_pop();

  /* create a image to see result of template */
  if (ibuf->float_buffer.data) {
    GPU_offscreen_read_color(offscreen, GPU_DATA_FLOAT, ibuf->float_buffer.data);
  }
  else if (ibuf->byte_buffer.data) {
    GPU_offscreen_read_color(offscreen, GPU_DATA_UBYTE, ibuf->byte_buffer.data);
  }
  if (ibuf->float_buffer.data && ibuf->byte_buffer.data) {
    IMB_rect_from_float(ibuf);
  }

  Image *ima = BKE_image_add_from_imbuf(&bmain, ibuf, "Grease Pencil Fill");
  ima->id.tag |= LIB_TAG_DOIT;

  BKE_image_release_ibuf(ima, ibuf, nullptr);

  /* Switch back to window-system-provided frame-buffer. */
  GPU_offscreen_unbind(offscreen, true);
  GPU_offscreen_free(offscreen);

  restore_region();

  return ima;
}

static bool do_frame_fill(Main &bmain,
                          ARegion &region,
                          const Scene &scene,
                          Depsgraph &depsgraph,
                          View3D &view3d,
                          const RegionView3D &rv3d,
                          const float pixel_scale,
                          const float2 zoom,
                          const float2 offset,
                          const bool invert,
                          ReportList &reports,
                          const bool keep_images)
{
  Image *ima = render_offscreen(
      bmain, region, scene, depsgraph, view3d, rv3d, pixel_scale, zoom, offset, reports);
  if (ima == nullptr) {
    return false;
  }

  //   wmWindow *win = CTX_wm_window(tgpf->C);

  //   int totpoints = 1;

  //     /* Set red borders to create a external limit. */
  //     gpencil_set_borders(tgpf, true);

  //     /* apply boundary fill */
  //     const bool border_contact = gpencil_boundaryfill_area(tgpf);

  //     /* Fill only if it never comes in contact with an edge. It is better not to fill than
  //      * to fill the entire area, as this is confusing for the artist. */
  //     if ((!border_contact) || (is_inverted)) {
  //       /* Invert direction if press Ctrl. */
  //       if (is_inverted) {
  //         gpencil_invert_image(tgpf);
  //         while (gpencil_find_and_mark_empty_areas(tgpf)) {
  //           gpencil_boundaryfill_area(tgpf);
  //           if (FILL_DEBUG) {
  //             break;
  //           }
  //         }
  //       }

  //       /* Clean borders to avoid infinite loops. */
  //       gpencil_set_borders(tgpf, false);
  //       WM_cursor_time(win, 50);
  //       int totpoints_prv = 0;
  //       int loop_limit = 0;
  //       while (totpoints > 0) {
  //         /* Analyze outline. */
  //         gpencil_get_outline_points(tgpf, (totpoints == 1) ? true : false);

  //         /* Create array of points from stack. */
  //         totpoints = gpencil_points_from_stack(tgpf);
  //         if (totpoints > 0) {
  //           /* Create z-depth array for reproject. */
  //           gpencil_get_depth_array(tgpf);

  //           /* Create stroke and reproject. */
  //           gpencil_stroke_from_buffer(tgpf);
  //         }
  //         if (is_inverted) {
  //           gpencil_erase_processed_area(tgpf);
  //         }
  //         else {
  //           /* Exit of the loop. */
  //           totpoints = 0;
  //         }

  //         /* free temp stack data */
  //         if (tgpf->stack) {
  //           BLI_stack_free(tgpf->stack);
  //         }
  //         WM_cursor_time(win, 100);

  //         /* Free memory. */
  //         MEM_SAFE_FREE(tgpf->sbuffer);
  //         MEM_SAFE_FREE(tgpf->depth_arr);

  //         /* Limit very small areas. */
  //         if (totpoints < 3) {
  //           break;
  //         }
  //         /* Limit infinite loops is some corner cases. */
  //         if (totpoints_prv == totpoints) {
  //           loop_limit++;
  //           if (loop_limit > 3) {
  //             break;
  //           }
  //         }
  //         totpoints_prv = totpoints;
  //       }
  //     }
  //     else {
  //       BKE_report(tgpf->reports, RPT_INFO, "Unable to fill unclosed areas");
  //     }

  /* Delete temp image. */
  if (!keep_images) {
    BKE_id_free(&bmain, ima);
  }

  return true;
}

bool fill_strokes(bContext &C,
                  ARegion &region,
                  const VArray<bool> &is_boundary_layer,
                  const bool invert,
                  ReportList &reports)
{
  wmWindow &win = *CTX_wm_window(&C);
  View3D &view3d = *CTX_wm_view3d(&C);
  RegionView3D &rv3d = *CTX_wm_region_view3d(&C);
  Main &bmain = *CTX_data_main(&C);
  const Scene &scene = *CTX_data_scene(&C);
  Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &object = *CTX_data_active_object(&C);
  const Object &object_eval = *DEG_get_evaluated_object(&depsgraph, &object);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object.data);
  const ToolSettings &ts = *CTX_data_tool_settings(&C);
  const Brush &brush = *BKE_paint_brush(&ts.gp_paint->paint);
  //   auto &op_data = *static_cast<GreasePencilFillOpData *>(op.customdata);
  //   const bool extend_lines = (op_data.fill_extend_fac > 0.0f);

  /* Debug setting: keep image data blocks for inspection. */
  constexpr const bool keep_images = true;

  if (region.regiontype != RGN_TYPE_WINDOW) {
    return false;
  }
  if (!grease_pencil.has_active_layer()) {
    return false;
  }

  /* Drawings that form boundaries for the generated strokes. */
  const Vector<DrawingInfo> src_drawings = ed::greasepencil::retrieve_visible_drawings(
      scene, grease_pencil, false);
  /* Drawings from the active layer where strokes are generated. */
  const Vector<MutableDrawingInfo> dst_drawings =
      ed::greasepencil::retrieve_editable_drawings_from_layer(
          scene, grease_pencil, *grease_pencil.get_active_layer());

  // TODO based on the fill_factor (aka "Precision") setting.
  const float pixel_scale = 1.0f;

  for (const MutableDrawingInfo &dst_info : dst_drawings) {
    bke::MutableAttributeAccessor attributes =
        dst_info.drawing.strokes_for_write().attributes_for_write();
    /* Zoom and offset based on bounds, to fit all strokes within the render. */
    const rctf bounds = get_boundary_bounds(
        region, brush, object, object_eval, src_drawings, is_boundary_layer);
    const rctf region_bounds = get_region_bounds(region);
    const float2 bounds_max = float2(bounds.xmax, bounds.ymax);
    const float2 bounds_min = float2(bounds.xmax, bounds.ymax);
    const float2 region_max = float2(region_bounds.xmax, region_bounds.ymax);
    const float2 region_min = float2(region_bounds.xmax, region_bounds.ymax);
    const float2 zoom = math::safe_divide(region_max - region_min, bounds_max - bounds_min);
    const float2 offset = 0.5f * ((region_min + region_max) - (bounds_min + bounds_max) * zoom);

    do_frame_fill(bmain,
                  region,
                  scene,
                  depsgraph,
                  view3d,
                  rv3d,
                  pixel_scale,
                  zoom,
                  offset,
                  invert,
                  reports,
                  keep_images);
  }

  // tgpf->mouse[0] = event->mval[0];
  // tgpf->mouse[1] = event->mval[1];
  // tgpf->is_render = true;

  // /* Create Temp stroke. */
  // tgpf->gps_mouse = BKE_gpencil_stroke_new(0, 1, 10.0f);
  // tGPspoint point2D;
  // bGPDspoint *pt = &tgpf->gps_mouse->points[0];
  // copy_v2fl_v2i(point2D.m_xy, tgpf->mouse);
  // gpencil_stroke_convertcoords_tpoint(
  //     tgpf->scene, tgpf->region, tgpf->ob, &point2D, nullptr, &pt->x);

  // /* Hash of selected frames. */
  // GHash *frame_list = BLI_ghash_int_new_ex(__func__, 64);

  // /* If not multi-frame and there is no frame in scene->r.cfra for the active layer,
  //  * create a new frame. */
  // if (!is_multiedit) {
  //   tgpf->gpf = BKE_gpencil_layer_frame_get(
  //       tgpf->gpl,
  //       tgpf->active_cfra,
  //       blender::animrig::is_autokey_on(tgpf->scene) ? GP_GETFRAME_ADD_NEW :
  //       GP_GETFRAME_USE_PREV);
  //   tgpf->gpf->flag |= GP_FRAME_SELECT;

  //   BLI_ghash_insert(frame_list, POINTER_FROM_INT(tgpf->active_cfra), tgpf->gpl->actframe);
  // }
  // else {
  //   BKE_gpencil_frame_selected_hash(tgpf->gpd, frame_list);
  // }

  // /* Loop all frames. */
  // wmWindow *win = CTX_wm_window(C);
  // GHashIterator gh_iter;
  // int total = BLI_ghash_len(frame_list);
  // int i = 1;
  // GHASH_ITER (gh_iter, frame_list) {
  //   /* Set active frame as current for filling. */
  //   tgpf->active_cfra = POINTER_AS_INT(BLI_ghashIterator_getKey(&gh_iter));
  //   int step = (float(i) / float(total)) * 100.0f;
  //   WM_cursor_time(win, step);

  //   if (extend_lines) {
  //     grease_pencil_update_extend(C, op_data);
  //   }

  //   /* Repeat loop until get something. */
  //   tgpf->done = false;
  //   int loop_limit = 0;
  //   while ((!tgpf->done) && (loop_limit < 2)) {
  //     WM_cursor_time(win, loop_limit + 1);
  //     /* Render screen to temp image and do fill. */
  //     gpencil_do_frame_fill(tgpf, is_inverted);

  //     /* restore size */
  //     tgpf->region->winx = short(tgpf->bwinx);
  //     tgpf->region->winy = short(tgpf->bwiny);
  //     tgpf->region->winrct = tgpf->brect;
  //     if (!tgpf->done) {
  //       /* If the zoom was not set before, avoid a loop. */
  //       if (tgpf->zoom == 1.0f) {
  //         loop_limit++;
  //       }
  //       else {
  //         tgpf->zoom = 1.0f;
  //         tgpf->fill_factor = max_ff(
  //             GPENCIL_MIN_FILL_FAC,
  //             min_ff(brush->gpencil_settings->fill_factor, GPENCIL_MAX_FILL_FAC));
  //       }
  //     }
  //     loop_limit++;
  //   }

  //   if (extend_lines) {
  //     stroke_array_free(tgpf);
  //     gpencil_delete_temp_stroke_extension(tgpf, true);
  //   }

  //   i++;
  // }

  WM_cursor_modal_restore(&win);
  /* Free hash table. */
  // BLI_ghash_free(frame_list, nullptr, nullptr);

  /* Free temp stroke. */
  // BKE_gpencil_free_stroke(tgpf->gps_mouse);

  /* push undo data */
  // gpencil_undo_push(tgpf->gpd);

  return true;
}

}  // namespace blender::ed::greasepencil
