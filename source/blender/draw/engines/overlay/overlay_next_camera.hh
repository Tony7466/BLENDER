/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_camera.h"
#include "BKE_movieclip.h"
#include "BKE_tracking.h"
#include "DEG_depsgraph_query.h"
#include "DNA_camera_types.h"
#include "ED_view3d.h"
#include "overlay_next_extra_passes.hh"

namespace blender::draw::overlay {

struct CameraInstanceData : public ExtraInstanceData {
  CameraInstanceData(const ExtraInstanceData &data) : ExtraInstanceData(data){};
  CameraInstanceData() : ExtraInstanceData(){};

  /* Pack render data inside matrix and color. */
  float &volume_start()
  {
    return color[2];
  }
  float &volume_end()
  {
    return color[3];
  }
  float &depth()
  {
    return color[3];
  }
  float &focus()
  {
    return color[3];
  }
  float &dist_color_id()
  {
    return matrix[0][3];
  }
  float &corner_x()
  {
    return matrix[0][3];
  }
  float &corner_y()
  {
    return matrix[1][3];
  }
  float &center_x()
  {
    return matrix[2][3];
  }
  float &clip_start()
  {
    return matrix[2][3];
  }
  float &mist_start()
  {
    return matrix[2][3];
  }
  float &center_y()
  {
    return matrix[3][3];
  }
  float &clip_end()
  {
    return matrix[3][3];
  }
  float &mist_end()
  {
    return matrix[3][3];
  }
};

static void camera_view3d_reconstruction(const ObjectRef &ob_ref,
                                         const select::ID select_id,
                                         const State &state,
                                         ExtraInstancePasses &passes,
                                         CameraInstanceData data)
{
  const Scene *scene = state.scene;
  const View3D *v3d = state.v3d;
  Object *ob = ob_ref.object;

  const bool is_select = state.selection_type != SelectionType::DISABLED;

  MovieClip *clip = BKE_object_movieclip_get((Scene *)scene, ob, false);
  if (clip == nullptr) {
    return;
  }

  const bool is_solid_bundle = (v3d->bundle_drawtype == OB_EMPTY_SPHERE) &&
                               ((v3d->shading.type != OB_SOLID) || !XRAY_FLAG_ENABLED(v3d));

  MovieTracking *tracking = &clip->tracking;
  /* Index must start in 1, to mimic BKE_tracking_track_get_for_selection_index. */
  int track_index = 1;

  float3 bundle_color_custom;
  /* TODO(Miguel Pozo) */
  float *bundle_color_solid = G_draw.block.color_bundle_solid;
  float *bundle_color_unselected = G_draw.block.color_wire;
  uchar text_color_selected[4], text_color_unselected[4];
  /* Color Management: Exception here as texts are drawn in sRGB space directly. */
  UI_GetThemeColor4ubv(TH_SELECT, text_color_selected);
  UI_GetThemeColor4ubv(TH_TEXT, text_color_unselected);

  float4x4 camera_mat;
  BKE_tracking_get_camera_object_matrix(ob, camera_mat.ptr());

  LISTBASE_FOREACH (MovieTrackingObject *, tracking_object, &tracking->objects) {
    float4x4 tracking_object_mat;

    if (tracking_object->flag & TRACKING_OBJECT_CAMERA) {
      tracking_object_mat = camera_mat;
    }
    else {
      const int framenr = BKE_movieclip_remap_scene_to_clip_frame(clip,
                                                                  DEG_get_ctime(state.depsgraph));

      float4x4 object_mat;
      BKE_tracking_camera_get_reconstructed_interpolate(
          tracking, tracking_object, framenr, object_mat.ptr());

      tracking_object_mat = data.matrix * math::invert(object_mat);
    }

    LISTBASE_FOREACH (MovieTrackingTrack *, track, &tracking_object->tracks) {
      if ((track->flag & TRACK_HAS_BUNDLE) == 0) {
        continue;
      }
      bool is_selected = TRACK_SELECTED(track);

      CameraInstanceData bundle_data = data.with_size(v3d->bundle_size);
      bundle_data.matrix = math::translate(bundle_data.matrix, float3(track->bundle_pos));

      if (track->flag & TRACK_CUSTOMCOLOR) {
        /* Meh, hardcoded srgb transform here. */
        /* TODO: change the actual DNA color to be linear. */
        srgb_to_linearrgb_v3_v3(bundle_data.color, track->color);
      }
      else if (is_solid_bundle) {
        bundle_data.color = bundle_color_solid;
      }
      else if (!is_selected) {
        bundle_data.color = bundle_color_unselected;
      }

      if (is_select) {
        /* TODO(Miguel Pozo):
        DRW_select_load_id(ob->runtime.select_id | (track_index << 16));
        */
        track_index++;
      }

      if (is_solid_bundle) {
        if (is_selected) {
          passes.empty_buf(v3d->bundle_drawtype)
              .append(bundle_data.with_color(data.color), select_id);
        }
        passes.sphere_solid.append(bundle_data, select_id);
      }
      else {
        passes.empty_buf(v3d->bundle_drawtype).append(bundle_data, select_id);
      }

      if ((v3d->flag2 & V3D_SHOW_BUNDLENAME) && !is_select) {
        /** TODO(Miguel Pozo):
        DRWTextStore *dt = DRW_text_cache_ensure();

        DRW_text_cache_add(dt,
                           bundle_mat[3],
                           track->name,
                           strlen(track->name),
                           10,
                           0,
                           DRW_TEXT_CACHE_GLOBALSPACE | DRW_TEXT_CACHE_STRING_PTR,
                           is_selected ? text_color_selected : text_color_unselected);
        */
      }
    }

    if ((v3d->flag2 & V3D_SHOW_CAMERAPATH) && (tracking_object->flag & TRACKING_OBJECT_CAMERA) &&
        !is_select)
    {
      const MovieTrackingReconstruction *reconstruction = &tracking_object->reconstruction;

      if (reconstruction->camnr) {
        const MovieReconstructedCamera *camera = reconstruction->cameras;
        float3 v0 = float3(0);
        float3 v1 = float3(0);
        for (int a = 0; a < reconstruction->camnr; a++, camera++) {
          v0 = v1;
          v1 = math::transform_point(camera_mat, float3(camera->mat[3]));
          if (a > 0) {
            /* This one is suboptimal (gl_lines instead of gl_line_strip)
             * but we keep this for simplicity */
            /** TODO(Miguel Pozo):
              OVERLAY_extra_line(cb, v0, v1, TH_CAMERA_PATH);
            */
          }
        }
      }
    }
  }
}

static float camera_offaxis_shiftx_get(const Scene *scene,
                                       Object *ob,
                                       CameraInstanceData data,
                                       bool right_eye)
{
  Camera *cam = static_cast<Camera *>(ob->data);
  if (cam->stereo.convergence_mode != CAM_S3D_OFFAXIS) {
    return 0.0;
  }
  const char *viewnames[2] = {STEREO_LEFT_NAME, STEREO_RIGHT_NAME};
  const float shiftx = BKE_camera_multiview_shift_x(&scene->r, ob, viewnames[right_eye]);
  const float delta_shiftx = shiftx - cam->shiftx;
  const float width = data.corner_x() * 2.0f;
  return delta_shiftx * width;
}

/**
 * Draw the stereo 3d support elements (cameras, plane, volume).
 * They are only visible when not looking through the camera:
 */
static void camera_stereoscopy_extra(const ObjectRef &ob_ref,
                                     const select::ID select_id,
                                     const State &state,
                                     ExtraInstancePasses &passes,
                                     CameraInstanceData data)
{
  CameraInstanceData stereodata = data;
  const View3D *v3d = state.v3d;
  const Scene *scene = state.scene;
  Object *ob = ob_ref.object;
  Camera *cam = static_cast<Camera *>(ob->data);
  const bool is_select = state.selection_type != SelectionType::DISABLED;
  const char *viewnames[2] = {STEREO_LEFT_NAME, STEREO_RIGHT_NAME};

  const bool is_stereo3d_cameras = (v3d->stereo3d_flag & V3D_S3D_DISPCAMERAS) != 0;
  const bool is_stereo3d_plane = (v3d->stereo3d_flag & V3D_S3D_DISPPLANE) != 0;
  const bool is_stereo3d_volume = (v3d->stereo3d_flag & V3D_S3D_DISPVOLUME) != 0;

  if (!is_stereo3d_cameras) {
    /* Draw single camera. */
    passes.camera_frame.append(data, select_id);
  }

  for (int eye : IndexRange(2)) {
    ob = BKE_camera_multiview_render(scene, ob, viewnames[eye]);
    BKE_camera_multiview_model_matrix(&scene->r, ob, viewnames[eye], stereodata.matrix.ptr());

    stereodata.corner_x() = data.corner_x();
    stereodata.corner_y() = data.corner_y();
    stereodata.center_x() = data.center_x() + camera_offaxis_shiftx_get(scene, ob, data, eye);
    stereodata.center_y() = data.center_y();
    stereodata.depth() = data.depth();

    if (is_stereo3d_cameras) {
      passes.camera_frame.append(stereodata, select_id);

      /* Connecting line between cameras. */
      /* TODO(Miguel Pozo)
      OVERLAY_extra_line_dashed(cb, stereodata.matrix.location(), data.matrix.location(),
      G_draw.block.color_wire);
      */
    }

    if (is_stereo3d_volume && !is_select) {
      float r = (eye == 1) ? 2.0f : 1.0f;

      stereodata.volume_start() = -cam->clip_start;
      stereodata.volume_end() = -cam->clip_end;
      /* Encode eye + intensity and alpha (see shader) */
      // stereodata.color.xy() = {r + 0.15f, 1.0f};
      copy_v2_v2(stereodata.color, float2(r + 0.15f, 1.0f));
      passes.camera_frame.append(stereodata, select_id);

      if (v3d->stereo3d_volume_alpha > 0.0f) {
        /* Encode eye + intensity and alpha (see shader) */
        // stereodata.color.xy() = {r + 0.999f, v3d->stereo3d_volume_alpha};
        copy_v2_v2(stereodata.color, float2(r + 0.999f, v3d->stereo3d_convergence_alpha));

        passes.camera_volume.append(stereodata, select_id);
      }
      /* restore */
      stereodata.color = data.color;
    }
  }

  if (is_stereo3d_plane && !is_select) {
    if (cam->stereo.convergence_mode == CAM_S3D_TOE) {
      /* There is no real convergence plane but we highlight the center
       * point where the views are pointing at. */
      // zero_v3(stereodata.matrix[0]); /* We reconstruct from Z and Y */
      // zero_v3(stereodata.matrix[1]); /* Y doesn't change */
      stereodata.matrix.z_axis() = float3(0.0f);
      stereodata.matrix.location() = float3(0.0f);
      for (int i : IndexRange(2)) {
        float4x4 mat;
        /* Need normalized version here. */
        BKE_camera_multiview_model_matrix(&scene->r, ob, viewnames[i], mat.ptr());
        stereodata.matrix.z_axis() += mat.z_axis();
        stereodata.matrix.location() += mat.location() * 0.5f;
      }
      stereodata.matrix.z_axis() = math::normalize(stereodata.matrix.z_axis());
      stereodata.matrix.x_axis() = math::cross(stereodata.matrix.y_axis(),
                                               stereodata.matrix.z_axis());
    }
    else if (cam->stereo.convergence_mode == CAM_S3D_PARALLEL) {
      /* Show plane at the given distance between the views even if it makes no sense. */
      stereodata.matrix.location() = float3(0.0f);
      for (int i : IndexRange(2)) {
        float4x4 mat;
        BKE_camera_multiview_model_matrix_scaled(&scene->r, ob, viewnames[i], mat.ptr());
        stereodata.matrix.location() += mat.location() * 0.5f;
      }
    }
    else if (cam->stereo.convergence_mode == CAM_S3D_OFFAXIS) {
      /* Nothing to do. Everything is already setup. */
    }
    stereodata.volume_start() = -cam->stereo.convergence_distance;
    stereodata.volume_end() = -cam->stereo.convergence_distance;
    /* Encode eye + intensity and alpha (see shader) */
    // stereodata.color.xy() = {0.1f, 1.0f};
    copy_v2_v2(stereodata.color, float2(0.1f, 1.0f));
    passes.camera_frame.append(stereodata, select_id);

    if (v3d->stereo3d_convergence_alpha > 0.0f) {
      /* Encode eye + intensity and alpha (see shader) */
      // stereodata.color.xy() = {0.0f, v3d->stereo3d_convergence_alpha};
      copy_v2_v2(stereodata.color, float2(0.0f, v3d->stereo3d_convergence_alpha));
      passes.camera_volume.append(stereodata, select_id);
    }
  }
}

static void camera_sync(const ObjectRef &ob_ref,
                        const select::ID select_id,
                        Resources & /*res*/,
                        const State &state,
                        ExtraInstancePasses &passes,
                        CameraInstanceData data)
{
  Object *ob = ob_ref.object;

  const ViewLayer *view_layer = state.view_layer;
  const View3D *v3d = state.v3d;
  const Scene *scene = state.scene;
  const RegionView3D *rv3d = state.rv3d;

  Camera *cam = static_cast<Camera *>(ob->data);
  Object *camera_object = DEG_get_evaluated_object(state.depsgraph, v3d->camera);
  const bool is_select = state.selection_type != SelectionType::DISABLED;
  const bool is_active = (ob == camera_object);
  const bool look_through = (is_active && (rv3d->persp == RV3D_CAMOB));

  const bool is_multiview = (scene->r.scemode & R_MULTIVIEW) != 0;
  const bool is_stereo3d_view = (scene->r.views_format == SCE_VIEWS_FORMAT_STEREO_3D);
  const bool is_stereo3d_display_extra = is_active && is_multiview && (!look_through) &&
                                         ((v3d->stereo3d_flag) != 0);
  const bool is_selection_camera_stereo = is_select && look_through && is_multiview &&
                                          is_stereo3d_view;

  float3 scale = math::to_scale(data.matrix);
  /* BKE_camera_multiview_model_matrix already accounts for scale, don't do it here. */
  if (is_selection_camera_stereo) {
    scale = float3(1.0f);
  }
  else if (ELEM(0.0f, scale.x, scale.y, scale.z)) {
    /* Avoid division by 0. */
    return;
  }

  data.matrix.view<3, 3>() = math::normalize(data.matrix.view<3, 3>());

  float3 scale_inv = 1.0f / scale;
  float4x3 vecs;
  float2 aspect_ratio;
  float2 shift;
  float drawsize;
  BKE_camera_view_frame_ex(scene,
                           cam,
                           cam->drawsize,
                           look_through,
                           scale_inv,
                           aspect_ratio,
                           shift,
                           &drawsize,
                           vecs.ptr());

  /* Apply scale to simplify the rest of the drawing. */
  for (int i : IndexRange(4)) {
    vecs[i] *= scale;
    /* Project to z=-1 plane. Makes positioning / scaling easier. (see shader) */
    // vecs[i].xy() *= 1.0f / std::abs(vecs[i].z);
    mul_v2_fl(vecs[i], 1.0f / std::abs(vecs[i].z));
  }

  /* Frame coords */
  float2 center = (vecs[0].xy() + vecs[2].xy()) / 2.0f;
  float2 corner = vecs[0].xy() - center;

  data.corner_x() = corner.x;
  data.corner_y() = corner.y;
  data.center_x() = center.x;
  data.center_y() = center.y;
  data.depth() = vecs[0].z;

  if (look_through) {
    /* TODO(Miguel Pozo) */
    if (!DRW_state_is_image_render()) {
      /* Only draw the frame. */
      if (is_multiview) {
        float4x4 mat;
        const bool is_right = v3d->multiview_eye == STEREO_RIGHT_ID;
        const char *view_name = is_right ? STEREO_RIGHT_NAME : STEREO_LEFT_NAME;
        BKE_camera_multiview_model_matrix(&scene->r, ob, view_name, mat.ptr());
        data.center_x() += camera_offaxis_shiftx_get(scene, ob, data, is_right);
        for (int i : IndexRange(4)) {
          /* Partial copy to avoid overriding packed data. */
          // data.matrix[i].xyz() = mat[i].xyz();
          copy_v3_v3(data.matrix[i], mat[i].xyz());
        }
      }
      data.depth() *= -1.0f; /* Hides the back of the camera wires (see shader). */
      passes.camera_frame.append(data, select_id);
    }
  }
  else {
    /* Stereo cameras, volumes, plane drawing. */
    if (is_stereo3d_display_extra) {
      camera_stereoscopy_extra(ob_ref, select_id, state, passes, data);
    }
    else {
      passes.camera_frame.append(data, select_id);
    }
  }

  if (!look_through) {
    /* Triangle. */
    float tria_size = 0.7f * drawsize / std::abs(data.depth());
    float tria_margin = 0.1f * drawsize / std::abs(data.depth());
    data.center_x() = center.x;
    data.center_y() = center.y + data.corner_y() + tria_margin + tria_size;
    data.corner_x() = data.corner_y() = -tria_size;
    ExtraInstanceBuf &buf = is_active ? passes.camera_tria : passes.camera_tria_wire;
    buf.append(data, select_id);
  }

  if (cam->flag & CAM_SHOWLIMITS) {
    /* Scale focus point. */
    data.matrix.x_axis() *= cam->drawsize;
    data.matrix.y_axis() *= cam->drawsize;

    data.dist_color_id() = (is_active) ? 3 : 2;
    data.focus() = -BKE_camera_object_dof_distance(ob);
    data.clip_start() = cam->clip_start;
    data.clip_end() = cam->clip_end;
    passes.camera_distances.append(data, select_id);
  }

  if (cam->flag & CAM_SHOWMIST) {
    World *world = scene->world;
    if (world) {
      data.dist_color_id() = (is_active) ? 1 : 0;
      data.focus() = 1.0f; /* Disable */
      data.mist_start() = world->miststa;
      data.mist_end() = world->miststa + world->mistdist;
      passes.camera_distances.append(data, select_id);
    }
  }
#if 0
    /* Motion Tracking. */
    if ((v3d->flag2 & V3D_SHOW_RECONSTRUCTION) != 0) {
      camera_view3d_reconstruction(passes, scene, v3d, ob, data.color);
    }

    /* Background images. */
    if (look_through && (cam->flag & CAM_SHOW_BG_IMAGE) && !BLI_listbase_is_empty(&cam->bg_images))
    {
      OVERLAY_image_camera_cache_populate(vedata, ob);
    }
#endif
}

}  // namespace blender::draw::overlay
