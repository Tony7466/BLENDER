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

 private:
  /* Images added by Image > Background. Both added in preset view (like Top, Front, ..) and in
   * custom view. Object property "In Front" unchecked. */
  PassSortable empties_back_ps_ = {"empties_back_ps_"};
  /* All Empty images from cases of `empties_ps_`, `empties_blend_ps_`, `empties_back_ps_`
   * with object property "In Front" checked. */
  PassSortable empties_front_ps_ = {"empties_front_ps_"};

  /* Images added by Empty > Image and Image > Reference with unchecked image "Opacity".
   * Object property "In Front" unchecked. */
  PassMain empties_ps_ = {"empties_ps_"};
  /* Images added by Empty > Image and Image > Reference with image "Opacity" checked.
   * Object property "In Front" unchecked. */
  PassSortable empties_blend_ps_ = {"empties_blend_ps_"};

  Vector<MovieClip *> bg_movie_clips;

  View view_reference_images = {"view_reference_images"};
  float view_dist = 0.0f;

  float3 camera_position;
  float3 camera_forward;

 public:
  class PassSource {
    const bool in_front;
    Images &images;

   public:
    PassSource(Images &images, bool in_front) : in_front(in_front), images(images) {}

    PassMain::Sub &operator()(const Object &ob,
                              const bool use_alpha_blend,
                              const float4x4 &mat) const
    {
      if (in_front) {
        return Images::create_subpass(
            images.camera_position, images.camera_forward, mat, images.empties_front_ps_);
      }
      const char depth_mode = DRW_state_is_depth() ? char(OB_EMPTY_IMAGE_DEPTH_DEFAULT) :
                                                     ob.empty_image_depth;
      switch (depth_mode) {
        case OB_EMPTY_IMAGE_DEPTH_BACK:
          return Images::create_subpass(
              images.camera_position, images.camera_forward, mat, images.empties_back_ps_);
        case OB_EMPTY_IMAGE_DEPTH_FRONT:
          return Images::create_subpass(
              images.camera_position, images.camera_forward, mat, images.empties_front_ps_);
        case OB_EMPTY_IMAGE_DEPTH_DEFAULT:
        default:
          return use_alpha_blend ? Images::create_subpass(images.camera_position,
                                                          images.camera_forward,
                                                          mat,
                                                          images.empties_blend_ps_) :
                                   images.empties_ps_;
      }
    }
  };

  const PassSource pass_source;
  const PassSource in_front_pass_source;

  Images() : pass_source(*this, false), in_front_pass_source(*this, true) {}

  ~Images() {}

  void begin_sync(Resources &res, const State &state, const View &view)
  {
    view_dist = state.view_dist_get(view.winmat());
    camera_position = view.viewinv().location();
    camera_forward = view.viewinv().z_axis();

    auto init_pass = [&](PassMain &pass, DRWState draw_state) {
      pass.init();
      pass.state_set(draw_state | state.clipping_state);
      pass.shader_set(res.shaders.image.get());
      res.select_bind(pass);
    };

    auto init_sortable = [&](PassSortable &pass, DRWState draw_state) {
      pass.init();
      pass.state_set(draw_state | state.clipping_state);
      pass.shader_set(res.shaders.image.get());
      res.select_bind(pass);
    };

    DRWState draw_state;

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS;
    init_pass(empties_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA_PREMUL;
    init_sortable(empties_back_ps_, draw_state);
    init_sortable(empties_blend_ps_, draw_state);

    draw_state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL;
    init_sortable(empties_front_ps_, draw_state);
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
      int2 size = int2(0);
      if (ima != nullptr) {
        ImageUser iuser = *ob->iuser;
        camera_background_images_stereo_setup(state.scene, state.v3d, ima, &iuser);
        tex = BKE_image_get_gpu_texture(ima, &iuser);
        if (tex) {
          size = int2(GPU_texture_original_width(tex), GPU_texture_original_height(tex));
        }
      }
      CLAMP_MIN(size.x, 1);
      CLAMP_MIN(size.y, 1);

      float2 image_aspect;
      overlay_image_calc_aspect(ima, size, image_aspect);

      mat = ob->object_to_world();
      mat.x_axis() *= image_aspect.x * 0.5f * ob->empty_drawsize;
      mat.y_axis() *= image_aspect.y * 0.5f * ob->empty_drawsize;
      mat[3] += float4(mat.x_axis() * (ob->ima_ofs[0] * 2.0f + 1.0f) +
                       mat.y_axis() * (ob->ima_ofs[1] * 2.0f + 1.0f));
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
      PassMain::Sub &pass = pass_source(*ob, use_alpha_blend, mat);
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

  void draw_image_background(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(empties_back_ps_, view);
  }

  void draw_image(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);

    view_reference_images.sync(view.viewmat(),
                               winmat_polygon_offset(view.winmat(), view_dist, -1.0f));

    manager.submit(empties_ps_, view_reference_images);
    manager.submit(empties_blend_ps_, view_reference_images);
  }

  void draw_in_front(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);

    view_reference_images.sync(view.viewmat(),
                               winmat_polygon_offset(view.winmat(), view_dist, -1.0f));

    manager.submit(empties_front_ps_, view_reference_images);
  }

 private:
  static PassMain::Sub &create_subpass(const float3 &camera_position,
                                       const float3 &camera_forward,
                                       const float4x4 &mat,
                                       PassSortable &parent)
  {
    const float3 tmp = camera_position - mat.location();
    const float z = -math::dot(camera_forward, tmp);
    return parent.sub("Sub", z);
  };

  static void overlay_image_calc_aspect(::Image *ima, const int2 &size, float2 &r_image_aspect)
  {
    /* if no image, make it a 1x1 empty square, honor scale & offset */
    const float2 ima_dim = ima ? float2(size.x, size.y) : float2(1.0f);

    /* Get the image aspect even if the buffer is invalid */
    float2 sca(1.0f);
    if (ima) {
      if (ima->aspx > ima->aspy) {
        sca.y = ima->aspy / ima->aspx;
      }
      else if (ima->aspx < ima->aspy) {
        sca.x = ima->aspx / ima->aspy;
      }
    }

    const float2 scale_inv(ima_dim.x * sca.x, ima_dim.y * sca.y);
    r_image_aspect = (scale_inv.x > scale_inv.y) ? float2(1.0f, scale_inv.y / scale_inv.x) :
                                                   float2(scale_inv.x / scale_inv.y, 1.0f);
  }

 public:
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
};

}  // namespace blender::draw::overlay
