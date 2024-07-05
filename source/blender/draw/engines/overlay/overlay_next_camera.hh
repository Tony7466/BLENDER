/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_camera.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_camera_types.h"

#include "overlay_next_private.hh"

static float camera_offaxis_shiftx_get(const Scene *scene,
                                       const Object *ob,
                                       float corner_x,
                                       bool right_eye)
{
  const Camera *cam = static_cast<const Camera *>(ob->data);
  if (cam->stereo.convergence_mode == CAM_S3D_OFFAXIS) {
    const char *viewnames[2] = {STEREO_LEFT_NAME, STEREO_RIGHT_NAME};
    const float shiftx = BKE_camera_multiview_shift_x(&scene->r, ob, viewnames[right_eye]);
    const float delta_shiftx = shiftx - cam->shiftx;
    const float width = corner_x * 2.0f;
    return delta_shiftx * width;
  }

  return 0.0;
}

namespace blender::draw::overlay {
struct CameraInstanceData : public ExtraInstanceData {
 public:
  float &volume_start = color_[2];
  float &volume_end = color_[3];
  float &depth = color_[3];
  float &focus = color_[3];
  float4x4 &matrix = object_to_world_;
  float &dist_color_id = matrix[0][3];
  float &corner_x = matrix[0][3];
  float &corner_y = matrix[1][3];
  float &center_x = matrix[2][3];
  float &clip_start = matrix[2][3];
  float &mist_start = matrix[2][3];
  float &center_y = matrix[3][3];
  float &clip_end = matrix[3][3];
  float &mist_end = matrix[3][3];

  CameraInstanceData(const CameraInstanceData &data)
      : CameraInstanceData(data.object_to_world_, data.color_)
  {
  }

  CameraInstanceData(const float4x4 &p_matrix, const float4 &color)
      : ExtraInstanceData(p_matrix, color, 1.0f){};
};

using CameraInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

class Cameras {
 private:
  const SelectionType selection_type_;

  PassSimple camera_ps_ = {"Cameras"};
  PassSimple camera_in_front_ps_ = {"Cameras_In_front"};
  struct CallBuffers {
    const SelectionType selection_type_;
    CameraInstanceBuf distances_buf = {selection_type_, "camera_distances_buf"};
    CameraInstanceBuf frame_buf = {selection_type_, "camera_frame_buf"};
    CameraInstanceBuf tria_buf = {selection_type_, "camera_tria_buf"};
    CameraInstanceBuf tria_wire_buf = {selection_type_, "camera_tria_wire_buf"};
    CameraInstanceBuf volume_buf = {selection_type_, "camera_volume_buf"};
    CameraInstanceBuf volume_wire_buf = {selection_type_, "camera_volume_wire_buf"};
    LineInstanceBuf stereo_connect_lines = {selection_type_, "camera_dashed_lines_buf"};
  } call_buffers_[2] = {{selection_type_}, {selection_type_}};

  /**
   * Draw the stereo 3d support elements (cameras, plane, volume).
   * They are only visible when not looking through the camera:
   */
  void stereoscopy_extra(const CameraInstanceData &instdata,
                         const select::ID select_id,
                         const Scene *scene,
                         const View3D *v3d,
                         Resources &res,
                         Object *ob,
                         CallBuffers &call_buffers)
  {
    CameraInstanceData stereodata = instdata;

    const Camera *cam = static_cast<const Camera *>(ob->data);
    const bool is_select = DRW_state_is_select();
    const char *viewnames[2] = {STEREO_LEFT_NAME, STEREO_RIGHT_NAME};

    const bool is_stereo3d_cameras = (v3d->stereo3d_flag & V3D_S3D_DISPCAMERAS) != 0;
    const bool is_stereo3d_plane = (v3d->stereo3d_flag & V3D_S3D_DISPPLANE) != 0;
    const bool is_stereo3d_volume = (v3d->stereo3d_flag & V3D_S3D_DISPVOLUME) != 0;

    if (!is_stereo3d_cameras) {
      /* Draw single camera. */
      call_buffers.frame_buf.append(instdata, select_id);
    }

    for (int eye = 0; eye < 2; eye++) {
      ob = BKE_camera_multiview_render(scene, ob, viewnames[eye]);
      BKE_camera_multiview_model_matrix(&scene->r, ob, viewnames[eye], stereodata.matrix.ptr());

      stereodata.corner_x = instdata.corner_x;
      stereodata.corner_y = instdata.corner_y;
      stereodata.center_x = instdata.center_x +
                            camera_offaxis_shiftx_get(scene, ob, instdata.corner_x, eye);
      stereodata.center_y = instdata.center_y;
      stereodata.depth = instdata.depth;

      if (is_stereo3d_cameras) {
        call_buffers.frame_buf.append(stereodata, select_id);

        /* Connecting line between cameras. */
        call_buffers.stereo_connect_lines.append(stereodata.matrix.location(),
                                                 instdata.object_to_world_.location(),
                                                 res.theme_settings.color_wire,
                                                 select_id);
      }

      if (is_stereo3d_volume && !is_select) {
        float r = (eye == 1) ? 2.0f : 1.0f;

        stereodata.volume_start = -cam->clip_start;
        stereodata.volume_end = -cam->clip_end;
        /* Encode eye + intensity and alpha (see shader) */
        copy_v2_fl2(stereodata.color_, r + 0.15f, 1.0f);
        call_buffers.volume_wire_buf.append(stereodata, select_id);

        if (v3d->stereo3d_volume_alpha > 0.0f) {
          /* Encode eye + intensity and alpha (see shader) */
          copy_v2_fl2(stereodata.color_, r + 0.999f, v3d->stereo3d_volume_alpha);
          call_buffers.volume_buf.append(stereodata, select_id);
        }
        /* restore */
        copy_v3_v3(stereodata.color_, instdata.color_);
      }
    }

    if (is_stereo3d_plane && !is_select) {
      if (cam->stereo.convergence_mode == CAM_S3D_TOE) {
        /* There is no real convergence plane but we highlight the center
         * point where the views are pointing at. */
        // zero_v3(stereodata.mat[0]); /* We reconstruct from Z and Y */
        // zero_v3(stereodata.mat[1]); /* Y doesn't change */
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
      stereodata.volume_start = -cam->stereo.convergence_distance;
      stereodata.volume_end = -cam->stereo.convergence_distance;
      /* Encode eye + intensity and alpha (see shader) */
      copy_v2_fl2(stereodata.color_, 0.1f, 1.0f);
      call_buffers.volume_wire_buf.append(stereodata, select_id);

      if (v3d->stereo3d_convergence_alpha > 0.0f) {
        /* Encode eye + intensity and alpha (see shader) */
        copy_v2_fl2(stereodata.color_, 0.0f, v3d->stereo3d_convergence_alpha);
        call_buffers.volume_buf.append(stereodata, select_id);
      }
    }
  }

 public:
  Cameras(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync()
  {
    for (int i = 0; i < 2; i++) {
      call_buffers_[i].distances_buf.clear();
      call_buffers_[i].frame_buf.clear();
      call_buffers_[i].tria_buf.clear();
      call_buffers_[i].tria_wire_buf.clear();
      call_buffers_[i].volume_buf.clear();
      call_buffers_[i].volume_wire_buf.clear();
      call_buffers_[i].stereo_connect_lines.clear();
    }
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    Object *ob = ob_ref.object;
    CallBuffers &call_buffers = call_buffers_[(ob->dtx & OB_DRAW_IN_FRONT) != 0 ? 1 : 0];
    const select::ID select_id = res.select_id(ob_ref);
    CameraInstanceData data(ob->object_to_world(), res.object_wire_color(ob_ref, state));

    const View3D *v3d = state.v3d;
    const Scene *scene = state.scene;
    const RegionView3D *rv3d = state.rv3d;

    const Camera *cam = static_cast<Camera *>(ob->data);
    const Object *camera_object = DEG_get_evaluated_object(state.depsgraph, v3d->camera);
    const bool is_select = DRW_state_is_select();
    const bool is_active = (ob == camera_object);
    const bool is_camera_view = (is_active && (rv3d->persp == RV3D_CAMOB));

    const bool is_multiview = (scene->r.scemode & R_MULTIVIEW) != 0;
    const bool is_stereo3d_view = (scene->r.views_format == SCE_VIEWS_FORMAT_STEREO_3D);
    const bool is_stereo3d_display_extra = is_active && is_multiview && (!is_camera_view) &&
                                           ((v3d->stereo3d_flag) != 0);
    const bool is_selection_camera_stereo = is_select && is_camera_view && is_multiview &&
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
    float4x3 vecs;
    float2 aspect_ratio;
    float2 shift;
    float drawsize;

    BKE_camera_view_frame_ex(scene,
                             cam,
                             cam->drawsize,
                             is_camera_view,
                             1.0f / scale,
                             aspect_ratio,
                             shift,
                             &drawsize,
                             vecs.ptr());

    /* Apply scale to simplify the rest of the drawing. */
    for (int i = 0; i < 4; i++) {
      vecs[i] *= scale;
      /* Project to z=-1 plane. Makes positioning / scaling easier. (see shader) */
      mul_v2_fl(vecs[i], 1.0f / std::abs(vecs[i].z));
    }

    /* Frame coords */
    const float2 center = (vecs[0].xy() + vecs[2].xy()) * 0.5f;
    const float2 corner = vecs[0].xy() - center.xy();
    data.corner_x = corner.x;
    data.corner_y = corner.y;
    data.center_x = center.x;
    data.center_y = center.y;
    data.depth = vecs[0].z;

    if (is_camera_view) {
      /* TODO(Miguel Pozo) */
      if (!DRW_state_is_image_render()) {
        /* Only draw the frame. */
        if (is_multiview) {
          float4x4 mat;
          const bool is_right = v3d->multiview_eye == STEREO_RIGHT_ID;
          const char *view_name = is_right ? STEREO_RIGHT_NAME : STEREO_LEFT_NAME;
          BKE_camera_multiview_model_matrix(&scene->r, ob, view_name, mat.ptr());
          data.center_x += camera_offaxis_shiftx_get(scene, ob, data.corner_x, is_right);
          for (int i : IndexRange(4)) {
            /* Partial copy to avoid overriding packed data. */
            copy_v3_v3(data.matrix[i], mat[i].xyz());
          }
        }
        data.depth *= -1.0f; /* Hides the back of the camera wires (see shader). */
        call_buffers.frame_buf.append(data, select_id);
      }
    }
    else {
      /* Stereo cameras, volumes, plane drawing. */
      if (is_stereo3d_display_extra) {
        stereoscopy_extra(data, select_id, scene, v3d, res, ob, call_buffers);
      }
      else {
        call_buffers.frame_buf.append(data, select_id);
      }
    }

    if (!is_camera_view) {
      /* Triangle. */
      float tria_size = 0.7f * drawsize / fabsf(data.depth);
      float tria_margin = 0.1f * drawsize / fabsf(data.depth);
      data.center_x = center.x;
      data.center_y = center.y + data.corner_y + tria_margin + tria_size;
      data.corner_x = data.corner_y = -tria_size;
      (is_active ? call_buffers.tria_buf : call_buffers.tria_wire_buf).append(data, select_id);
    }

    if (cam->flag & CAM_SHOWLIMITS) {
      /* Scale focus point. */
      data.matrix.x_axis() *= cam->drawsize;
      data.matrix.y_axis() *= cam->drawsize;

      data.dist_color_id = (is_active) ? 3 : 2;
      data.focus = -BKE_camera_object_dof_distance(ob);
      data.clip_start = cam->clip_start;
      data.clip_end = cam->clip_end;
      call_buffers.distances_buf.append(data, select_id);
    }

    if (cam->flag & CAM_SHOWMIST) {
      World *world = scene->world;
      if (world) {
        data.dist_color_id = (is_active) ? 1 : 0;
        data.focus = 1.0f; /* Disable */
        data.mist_start = world->miststa;
        data.mist_end = world->miststa + world->mistdist;
        call_buffers.distances_buf.append(data, select_id);
      }
    }

    // /* Motion Tracking. */
    // if ((v3d->flag2 & V3D_SHOW_RECONSTRUCTION) != 0) {
    //   camera_view3d_reconstruction(cb, scene, v3d, ob, color_p);
    // }

    // /* Background images. */
    // if (look_through && (cam->flag & CAM_SHOW_BG_IMAGE) &&
    // !BLI_listbase_is_empty(&cam->bg_images))
    // {
    //   OVERLAY_image_camera_cache_populate(vedata, ob);
    // }
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    auto init_pass = [&](PassSimple &pass, CallBuffers &call_bufs, bool in_front) {
      pass.init();
      res.select_bind(pass);

      if (!in_front) {
        {
          PassSimple::Sub &sub_pass = pass.sub("volume");
          sub_pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA |
                             DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_CULL_BACK |
                             state.clipping_state);
          sub_pass.shader_set(res.shaders.extra_shape.get());
          sub_pass.bind_ubo("globalsBlock", &res.globals_buf);
          call_bufs.volume_buf.end_sync(sub_pass, shapes.camera_volume.get());
        }
        {
          PassSimple::Sub &sub_pass = pass.sub("volume_wire");
          sub_pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA |
                             DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_CULL_BACK |
                             state.clipping_state);
          sub_pass.shader_set(res.shaders.extra_shape.get());
          sub_pass.bind_ubo("globalsBlock", &res.globals_buf);
          call_bufs.volume_wire_buf.end_sync(sub_pass, shapes.camera_volume_wire.get());
        }
      }
      {
        PassSimple::Sub &sub_pass = pass.sub("camera_shapes");
        sub_pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                           DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
        sub_pass.shader_set(res.shaders.extra_shape.get());
        sub_pass.bind_ubo("globalsBlock", &res.globals_buf);
        call_bufs.distances_buf.end_sync(sub_pass, shapes.camera_distances.get());
        call_bufs.frame_buf.end_sync(sub_pass, shapes.camera_frame.get());
        call_bufs.tria_buf.end_sync(sub_pass, shapes.camera_tria.get());
        call_bufs.tria_wire_buf.end_sync(sub_pass, shapes.camera_tria_wire.get());
      }
      {
        PassSimple &sub_pass = pass;
        sub_pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                           DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
        sub_pass.shader_set(res.shaders.extra_wire.get());
        sub_pass.bind_ubo("globalsBlock", &res.globals_buf);
        call_bufs.stereo_connect_lines.end_sync(sub_pass);
      }
    };
    init_pass(camera_ps_, call_buffers_[0], false);
    init_pass(camera_in_front_ps_, call_buffers_[1], true);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(camera_ps_, view);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(camera_in_front_ps_, view);
  }
};

}  // namespace blender::draw::overlay
