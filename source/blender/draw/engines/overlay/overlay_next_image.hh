/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_math_rotation.h"

#include "DNA_image_types.h"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Images {
  friend class Empties;
  using ImageInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

  PassMain background_scene_ps_ = {"background_scene_ps_"};
  PassMain foreground_scene_ps_ = {"foreground_scene_ps_"};

  PassMain background_ps_ = {"background_ps_"};
  PassMain foreground_ps_ = {"foreground_ps_"};

  PassMain empties_back_ps_ = {"empties_back_ps_"};
  PassMain empties_front_ps_ = {"empties_front_ps_"};

  PassMain empties_ps_ = {"empties_ps_"};
  PassMain empties_blend_ps_ = {"empties_blend_ps_"};

 public:
  class PassSource {
    const bool in_front;
    Images &images;

   public:
    PassSource(Images &images, bool in_front) : in_front(in_front), images(images) {}

    PassMain *operator()(const Object &ob, const bool use_alpha_blend) const
    {
      if (in_front) {
        return &(images.empties_front_ps_);
      }
      const char depth_mode = DRW_state_is_depth() ? char(OB_EMPTY_IMAGE_DEPTH_DEFAULT) :
                                                     ob.empty_image_depth;
      switch (depth_mode) {
        case OB_EMPTY_IMAGE_DEPTH_BACK:
          return &(images.empties_back_ps_);
        case OB_EMPTY_IMAGE_DEPTH_FRONT:
          return &(images.empties_front_ps_);
        case OB_EMPTY_IMAGE_DEPTH_DEFAULT:
        default:
          return &((use_alpha_blend) ? images.empties_blend_ps_ : images.empties_ps_);
      }
    }

    PassMain *operator()(const bool is_foreground, const bool use_view_transform) const
    {
      return &(is_foreground ?
                   (use_view_transform ? images.foreground_scene_ps_ : images.foreground_ps_) :
                   (use_view_transform ? images.background_scene_ps_ : images.background_ps_));
    }
  };

  const PassSource pass_source;
  const PassSource in_front_pass_source;

  Images() : pass_source(*this, false), in_front_pass_source(*this, true) {}

  void begin_sync(Resources &res, const State &state)
  {
    auto init_pass = [&](PassMain &pass, DRWState draw_state) {
      pass.init();
      pass.state_set(draw_state | state.clipping_state);
      pass.shader_set(res.shaders.image.get());
      res.select_bind(pass);
    };
    DRWState draw_state;
    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_GREATER | DRW_STATE_BLEND_ALPHA_PREMUL;
    init_pass(background_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_UNDER_PREMUL;
    init_pass(background_scene_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS;
    init_pass(empties_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA_PREMUL;
    init_pass(empties_back_ps_, draw_state);
    init_pass(empties_blend_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL;
    init_pass(empties_front_ps_, draw_state);
    init_pass(foreground_ps_, draw_state);
    init_pass(foreground_scene_ps_, draw_state);
  }

  static void object_sync_camera(const ObjectRef &ob_ref,
                                 select::ID select_id,
                                 ShapeCache &shapes,
                                 Manager &manager,
                                 const State &state,
                                 const SelectionType selection_type,
                                 const PassSource &pass_source)
  {
    Object *ob = ob_ref.object;
    const Camera *cam = static_cast<Camera *>(ob->data);

    const bool show_frame = BKE_object_empty_image_frame_is_visible_in_view3d(ob, state.rv3d);

    if (!show_frame || selection_type != SelectionType::DISABLED) {
      return;
    }

    const bool stereo_eye = camera_background_images_stereo_eye(state.scene, state.v3d) ==
                            STEREO_LEFT_ID;
    const char *viewname = (stereo_eye == STEREO_LEFT_ID) ? STEREO_RIGHT_NAME : STEREO_LEFT_NAME;
    float4x4 modelmat;
    BKE_camera_multiview_model_matrix(&state.scene->r, ob, viewname, modelmat.ptr());

    for (const CameraBGImage *bgpic : ConstListBaseWrapper<CameraBGImage>(&cam->bg_images)) {
      if (bgpic->flag & CAM_BGIMG_FLAG_DISABLED) {
        continue;
      }

      float aspect = 1.0;
      bool use_alpha_premult;
      bool use_view_transform = false;
      float4x4 mat;

      /* retrieve the image we want to show, continue to next when no image could be found */
      GPUTexture *tex = image_camera_background_texture_get(
          bgpic, state, aspect, use_alpha_premult, use_view_transform);

      if (tex) {
        image_camera_background_matrix_get(cam, bgpic, state, aspect, mat);

        const bool is_foreground = (bgpic->flag & CAM_BGIMG_FLAG_FOREGROUND) != 0;
        /* Alpha is clamped just below 1.0 to fix background images to interfere with foreground
         * images. Without this a background image with 1.0 will be rendered on top of a
         * transparent foreground image due to the different blending modes they use. */
        const float4 color_premult_alpha{1.0f, 1.0f, 1.0f, std::min(bgpic->alpha, 0.999999f)};

        PassMain &pass = *pass_source(is_foreground, use_view_transform);
        pass.bind_texture("imgTexture", tex);
        pass.push_constant("imgPremultiplied", use_alpha_premult);
        pass.push_constant("imgAlphaBlend", true);
        pass.push_constant("isCameraBackground", true);
        pass.push_constant("depthSet", true);
        pass.push_constant("ucolor", color_premult_alpha);
        ResourceHandle res_handle = manager.resource_handle(mat);
        pass.draw(shapes.quad_solid.get(), res_handle, select_id.get());
      }
    }
  }

  static void object_sync(const ObjectRef &ob_ref,
                          select::ID select_id,
                          ShapeCache &shapes,
                          Manager &manager,
                          Resources &res,
                          const State &state,
                          const PassSource &pass_source,
                          ImageInstanceBuf &empty_image_buf)
  {
    Object *ob = ob_ref.object;
    GPUTexture *tex = nullptr;
    ::Image *ima = static_cast<::Image *>(ob_ref.object->data);
    float4x4 mat;

    const bool show_frame = BKE_object_empty_image_frame_is_visible_in_view3d(ob, state.rv3d);
    const bool show_image = show_frame &&
                            BKE_object_empty_image_data_is_visible_in_view3d(ob, state.rv3d);
    const bool use_alpha_blend = (ob_ref.object->empty_image_flag &
                                  OB_EMPTY_IMAGE_USE_ALPHA_BLEND) != 0;
    const bool use_alpha_premult = ima && (ima->alpha_mode == IMA_ALPHA_PREMUL);

    if (!show_frame) {
      return;
    }

    {
      /* Calling 'BKE_image_get_size' may free the texture. Get the size from 'tex' instead,
       * see: #59347 */
      int size[2] = {0};
      if (ima != nullptr) {
        ImageUser iuser = *ob->iuser;
        camera_background_images_stereo_setup(state.scene, state.v3d, ima, &iuser);
        tex = BKE_image_get_gpu_texture(ima, &iuser);
        if (tex) {
          size[0] = GPU_texture_original_width(tex);
          size[1] = GPU_texture_original_height(tex);
        }
      }
      CLAMP_MIN(size[0], 1);
      CLAMP_MIN(size[1], 1);

      float image_aspect[2];
      overlay_image_calc_aspect(ima, size, image_aspect);

      mat = ob->object_to_world();
      mat.x_axis() *= image_aspect[0] * 0.5f * ob->empty_drawsize;
      mat.y_axis() *= image_aspect[1] * 0.5f * ob->empty_drawsize;
      madd_v3_v3fl(mat[3], mat[0], ob->ima_ofs[0] * 2.0f + 1.0f);
      madd_v3_v3fl(mat[3], mat[1], ob->ima_ofs[1] * 2.0f + 1.0f);
    }

    if (show_frame) {
      const float4 color = res.object_wire_color(ob_ref, state);
      empty_image_buf.append(ExtraInstanceData(mat, color, 1.0f), select_id);
    }

    if (show_image && tex && ((ob->color[3] > 0.0f) || !use_alpha_blend)) {
      /* Use the actual depth if we are doing depth tests to determine the distance to the
       * object. */
      char depth_mode = DRW_state_is_depth() ? char(OB_EMPTY_IMAGE_DEPTH_DEFAULT) :
                                               ob->empty_image_depth;
      PassMain &pass = *pass_source(*ob, use_alpha_blend);
      pass.bind_texture("imgTexture", tex);
      pass.push_constant("imgPremultiplied", use_alpha_premult);
      pass.push_constant("imgAlphaBlend", use_alpha_blend);
      pass.push_constant("isCameraBackground", false);
      pass.push_constant("depthSet", depth_mode != OB_EMPTY_IMAGE_DEPTH_DEFAULT);
      pass.push_constant("ucolor", float4(ob->color));
      ResourceHandle res_handle = manager.resource_handle(mat);
      pass.draw(shapes.quad_solid.get(), res_handle, select_id.get());
    }
  }

  void draw_image_sccene_background(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (DRW_state_is_fbo()) {
      GPU_framebuffer_bind(framebuffer);

      manager.submit(background_scene_ps_, view);
      manager.submit(foreground_scene_ps_, view);
    }
  }

  void draw_image_background(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(background_ps_, view);
    manager.submit(empties_back_ps_, view);
  }

  void draw_image(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(empties_ps_, view);
    manager.submit(empties_blend_ps_, view);
  }

 private:
  static void overlay_image_calc_aspect(::Image *ima, const int size[2], float r_image_aspect[2])
  {
    float ima_x, ima_y;
    if (ima) {
      ima_x = size[0];
      ima_y = size[1];
    }
    else {
      /* if no image, make it a 1x1 empty square, honor scale & offset */
      ima_x = ima_y = 1.0f;
    }
    /* Get the image aspect even if the buffer is invalid */
    float sca_x = 1.0f, sca_y = 1.0f;
    if (ima) {
      if (ima->aspx > ima->aspy) {
        sca_y = ima->aspy / ima->aspx;
      }
      else if (ima->aspx < ima->aspy) {
        sca_x = ima->aspx / ima->aspy;
      }
    }

    const float scale_x_inv = ima_x * sca_x;
    const float scale_y_inv = ima_y * sca_y;
    if (scale_x_inv > scale_y_inv) {
      r_image_aspect[0] = 1.0f;
      r_image_aspect[1] = scale_y_inv / scale_x_inv;
    }
    else {
      r_image_aspect[0] = scale_x_inv / scale_y_inv;
      r_image_aspect[1] = 1.0f;
    }
  }

  static eStereoViews camera_background_images_stereo_eye(const Scene *scene, const View3D *v3d)
  {
    if ((scene->r.scemode & R_MULTIVIEW) == 0) {
      return STEREO_LEFT_ID;
    }
    if (v3d->stereo3d_camera != STEREO_3D_ID) {
      /* show only left or right camera */
      return eStereoViews(v3d->stereo3d_camera);
    }

    return eStereoViews(v3d->multiview_eye);
  }

  static void camera_background_images_stereo_setup(const Scene *scene,
                                                    const View3D *v3d,
                                                    ::Image *ima,
                                                    ImageUser *iuser)
  {
    if (BKE_image_is_stereo(ima)) {
      iuser->flag |= IMA_SHOW_STEREO;
      iuser->multiview_eye = camera_background_images_stereo_eye(scene, v3d);
      BKE_image_multiview_index(ima, iuser);
    }
    else {
      iuser->flag &= ~IMA_SHOW_STEREO;
    }
  }

  static void image_camera_background_matrix_get(const Camera *cam,
                                                 const CameraBGImage *bgpic,
                                                 const State &state,
                                                 const float image_aspect,
                                                 float4x4 &rmat)
  {
    float4x4 rotate, scale = float4x4::identity(), translate = float4x4::identity();

    axis_angle_to_mat4_single(rotate.ptr(), 'Z', -bgpic->rotation);

    /* Normalized Object space camera frame corners. */
    float cam_corners[4][3];
    BKE_camera_view_frame(state.scene, cam, cam_corners);
    float cam_width = fabsf(cam_corners[0][0] - cam_corners[3][0]);
    float cam_height = fabsf(cam_corners[0][1] - cam_corners[1][1]);
    float cam_aspect = cam_width / cam_height;

    if (bgpic->flag & CAM_BGIMG_FLAG_CAMERA_CROP) {
      /* Crop. */
      if (image_aspect > cam_aspect) {
        scale[0][0] *= cam_height * image_aspect;
        scale[1][1] *= cam_height;
      }
      else {
        scale[0][0] *= cam_width;
        scale[1][1] *= cam_width / image_aspect;
      }
    }
    else if (bgpic->flag & CAM_BGIMG_FLAG_CAMERA_ASPECT) {
      /* Fit. */
      if (image_aspect > cam_aspect) {
        scale[0][0] *= cam_width;
        scale[1][1] *= cam_width / image_aspect;
      }
      else {
        scale[0][0] *= cam_height * image_aspect;
        scale[1][1] *= cam_height;
      }
    }
    else {
      /* Stretch. */
      scale[0][0] *= cam_width;
      scale[1][1] *= cam_height;
    }

    translate[3][0] = bgpic->offset[0];
    translate[3][1] = bgpic->offset[1];
    translate[3][2] = cam_corners[0][2];
    if (cam->type == CAM_ORTHO) {
      translate[3].xy() *= cam->ortho_scale;
    }
    /* These lines are for keeping 2.80 behavior and could be removed to keep 2.79 behavior. */
    translate[3][0] *= min_ff(1.0f, cam_aspect);
    translate[3][1] /= max_ff(1.0f, cam_aspect) * (image_aspect / cam_aspect);
    /* quad is -1..1 so divide by 2. */
    scale[0][0] *= 0.5f * bgpic->scale * ((bgpic->flag & CAM_BGIMG_FLAG_FLIP_X) ? -1.0 : 1.0);
    scale[1][1] *= 0.5f * bgpic->scale * ((bgpic->flag & CAM_BGIMG_FLAG_FLIP_Y) ? -1.0 : 1.0);
    /* Camera shift. (middle of cam_corners) */
    translate[3][0] += (cam_corners[0][0] + cam_corners[2][0]) * 0.5f;
    translate[3][1] += (cam_corners[0][1] + cam_corners[2][1]) * 0.5f;

    rmat = translate * rotate * scale;
  }

  static GPUTexture *image_camera_background_texture_get(const CameraBGImage *bgpic,
                                                         const State state,
                                                         float &r_aspect,
                                                         bool &r_use_alpha_premult,
                                                         bool &r_use_view_transform)
  {
    ::Image *image = bgpic->ima;
    ImageUser *iuser = (ImageUser *)&bgpic->iuser;
    MovieClip *clip = nullptr;
    GPUTexture *tex = nullptr;
    float aspect_x, aspect_y;
    int width, height;
    int ctime = int(DEG_get_ctime(state.depsgraph));
    r_use_alpha_premult = false;
    r_use_view_transform = false;

    switch (bgpic->source) {
      case CAM_BGIMG_SOURCE_IMAGE: {
        if (image == nullptr) {
          return nullptr;
        }
        r_use_alpha_premult = (image->alpha_mode == IMA_ALPHA_PREMUL);
        r_use_view_transform = (image->flag & IMA_VIEW_AS_RENDER) != 0;

        BKE_image_user_frame_calc(image, iuser, ctime);
        if (image->source == IMA_SRC_SEQUENCE && !(iuser->flag & IMA_USER_FRAME_IN_RANGE)) {
          /* Frame is out of range, don't show. */
          return nullptr;
        }

        camera_background_images_stereo_setup(state.scene, state.v3d, image, iuser);

        iuser->scene = (Scene *)state.scene;
        tex = BKE_image_get_gpu_viewer_texture(image, iuser);
        iuser->scene = nullptr;

        if (tex == nullptr) {
          return nullptr;
        }

        width = GPU_texture_original_width(tex);
        height = GPU_texture_original_height(tex);

        aspect_x = bgpic->ima->aspx;
        aspect_y = bgpic->ima->aspy;
        break;
      }

      case CAM_BGIMG_SOURCE_MOVIE: {
        if (bgpic->flag & CAM_BGIMG_FLAG_CAMERACLIP) {
          if (state.scene->camera) {
            clip = BKE_object_movieclip_get((Scene *)state.scene, state.scene->camera, true);
          }
        }
        else {
          clip = bgpic->clip;
        }

        if (clip == nullptr) {
          return nullptr;
        }

        BKE_movieclip_user_set_frame((MovieClipUser *)&bgpic->cuser, ctime);
        tex = BKE_movieclip_get_gpu_texture(clip, (MovieClipUser *)&bgpic->cuser);
        if (tex == nullptr) {
          return nullptr;
        }

        aspect_x = clip->aspx;
        aspect_y = clip->aspy;
        r_use_view_transform = true;

        BKE_movieclip_get_size(clip, &bgpic->cuser, &width, &height);

        /* Save for freeing. */
        /* TODO: */
        // BLI_addtail(&pd->bg_movie_clips, BLI_genericNodeN(clip));
        break;
      }

      default:
        /* Unsupported type. */
        return nullptr;
    }

    r_aspect = (width * aspect_x) / (height * aspect_y);
    return tex;
  }
};

}  // namespace blender::draw::overlay
