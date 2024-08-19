/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Images {

 public:
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
