/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "camera.h"

#include "DNA_camera_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"

#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

static pxr::GfCamera set_gf_camera(bool is_ortho,
                                   pxr::GfMatrix4d transform,
                                   pxr::GfVec2f aperture,
                                   pxr::GfVec2f l_shift,
                                   pxr::GfRange1f clip_range)
{
  pxr::GfCamera gf_camera = pxr::GfCamera();

  pxr::GfCamera::Projection projection = is_ortho ? pxr::GfCamera::Projection::Orthographic :
                                                    pxr::GfCamera::Projection::Perspective;
  gf_camera.SetProjection(projection);
  gf_camera.SetHorizontalAperture(aperture[0]);
  gf_camera.SetVerticalAperture(aperture[1]);
  gf_camera.SetHorizontalApertureOffset(l_shift[0] * aperture[0]);
  gf_camera.SetVerticalApertureOffset(l_shift[1] * aperture[1]);
  gf_camera.SetClippingRange(clip_range);
  gf_camera.SetTransform(transform);

  return gf_camera;
};

pxr::GfCamera gf_camera(const Depsgraph *depsgraph,
                        const View3D *v3d,
                        const ARegion *region,
                        const RenderData *rd,
                        pxr::GfVec4f tile)
{
  CameraParams camera_params;
  const RegionView3D *region_data = (const RegionView3D *)region->regiondata;

  BKE_camera_params_init(&camera_params);
  BKE_camera_params_from_view3d(&camera_params, depsgraph, v3d, region_data);

  int camera_sensor_size = BKE_camera_sensor_size(
      camera_params.sensor_fit, camera_params.sensor_x, camera_params.sensor_y);
  camera_sensor_size *= 2.0f;

  float ratio = float(region->winx) / region->winy;
  int mode;
  pxr::GfRange1f clip_range;
  pxr::GfVec2f lens_shift;
  pxr::GfVec2f sensor_size;
  pxr::GfVec2f ortho_size;
  pxr::GfMatrix4d transform = gf_matrix_from_transform(region_data->viewmat).GetInverse();
  clip_range = pxr::GfRange1f(camera_params.clip_start, camera_params.clip_end);

  switch (region_data->persp) {
    case RV3D_PERSP:
      mode = CAM_PERSP;
      lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty);
      if (ratio > 1.0f) {
        sensor_size = pxr::GfVec2f(camera_sensor_size, camera_sensor_size / ratio);
      }
      else {
        sensor_size = pxr::GfVec2f(camera_sensor_size * ratio, camera_sensor_size);
      }
      break;
    case RV3D_ORTHO: {
      mode = CAM_ORTHO;
      lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty);
      float o_size = region_data->dist * camera_sensor_size / camera_params.lens;
      if (ratio > 1.0f) {
        ortho_size = pxr::GfVec2f(o_size, o_size / ratio);
      }
      else {
        ortho_size = pxr::GfVec2f(o_size * ratio, o_size);
      }
      break;
    }
    case RV3D_CAMOB: {
      BKE_camera_params_from_object(&camera_params, v3d->camera);
      int sensor_fit = BKE_camera_sensor_fit(
          camera_params.sensor_fit, rd->xasp * rd->xsch, rd->yasp * rd->ysch);
      switch (sensor_fit) {
        case CAMERA_SENSOR_FIT_VERT:
          lens_shift = pxr::GfVec2f(camera_params.shiftx / ratio, camera_params.shifty);
          break;
        case CAMERA_SENSOR_FIT_HOR:
          lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty * ratio);
          break;
        case CAMERA_SENSOR_FIT_AUTO:
          if (ratio > 1.0f) {
            lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty * ratio);
          }
          else {
            lens_shift = pxr::GfVec2f(camera_params.shiftx / ratio, camera_params.shifty);
          }
          break;
        default:
          BLI_assert_unreachable();
      }

      const Camera *camera = (const Camera *)v3d->camera->data;

      switch (camera->type) {
        case CAM_PANO:
        case CAM_PERSP:
          mode = camera->type == CAM_PERSP ? CAM_PERSP : CAM_PANO;
          switch (sensor_fit) {
            case CAMERA_SENSOR_FIT_VERT:
              sensor_size = pxr::GfVec2f(camera_params.sensor_y * ratio, camera_params.sensor_y);
              break;
            case CAMERA_SENSOR_FIT_HOR:
              sensor_size = pxr::GfVec2f(camera_params.sensor_x, camera_params.sensor_x / ratio);
              break;
            case CAMERA_SENSOR_FIT_AUTO:
              if (ratio > 1.0f) {
                sensor_size = pxr::GfVec2f(camera_params.sensor_x, camera_params.sensor_x / ratio);
              }
              else {
                sensor_size = pxr::GfVec2f(camera_params.sensor_x * ratio, camera_params.sensor_x);
              }
              break;
            default:
              BLI_assert_unreachable();
          }
          break;
        case CAM_ORTHO:
          mode = CAM_ORTHO;
          switch (sensor_fit) {
            case CAMERA_SENSOR_FIT_VERT:
              ortho_size = pxr::GfVec2f(camera_params.ortho_scale * ratio,
                                        camera_params.ortho_scale);
              break;
            case CAMERA_SENSOR_FIT_HOR:
              ortho_size = pxr::GfVec2f(camera_params.ortho_scale,
                                        camera_params.ortho_scale / ratio);
              break;
            case CAMERA_SENSOR_FIT_AUTO:
              if (ratio > 1.0f) {
                ortho_size = pxr::GfVec2f(camera_params.ortho_scale,
                                          camera_params.ortho_scale / ratio);
              }
              else {
                ortho_size = pxr::GfVec2f(camera_params.ortho_scale * ratio,
                                          camera_params.ortho_scale);
              }
              break;
            default:
              BLI_assert_unreachable();
          }
          break;
        default:
          BLI_assert_unreachable();
      }
      /* This formula was taken from previous plugin with corresponded comment.
       * See blender/intern/cycles/blender/blender_camera.cpp:blender_camera_from_view (look
       * for 1.41421f). */
      float zoom = 4.0 / pow((pow(2.0, 0.5) + region_data->camzoom / 50.0), 2);

      /* Updating l_shift due to viewport zoom and view_camera_offset
       * view_camera_offset should be multiplied by 2. */
      lens_shift = pxr::GfVec2f((lens_shift[0] + camera_params.offsetx * 2) / zoom,
                                (lens_shift[1] + camera_params.offsety * 2) / zoom);
      if (camera_params.is_ortho) {
        ortho_size *= zoom;
      }
      else {
        sensor_size *= zoom;
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }

  if (mode == CAM_PANO) {
    CLOG_WARN(LOG_HYDRA_SCENE, "Unsupported camera type: %d, perspective will be used", mode);
  }

  float t_pos[2] = {tile[0], tile[1]}, t_size[2] = {tile[2], tile[3]};

  lens_shift = {(lens_shift[0] + t_pos[0] + t_size[0] * 0.5f - 0.5f) / t_size[0],
                (lens_shift[1] + t_pos[1] + t_size[1] * 0.5f - 0.5f) / t_size[1]};

  pxr::GfVec2f aperture;

  if (camera_params.is_ortho) {
    /* Use tenths of a world unit according to USD docs
     * https://graphics.pixar.com/usd/docs/api/class_gf_camera.html */
    aperture[0] = ortho_size[0] * t_size[0] * 10;
    aperture[1] = ortho_size[1] * t_size[1] * 10;
  }
  else {
    aperture[0] = sensor_size[0] * t_size[0];
    aperture[1] = sensor_size[1] * t_size[1];
  }
  return set_gf_camera(camera_params.is_ortho, transform, aperture, lens_shift, clip_range);
}

pxr::GfCamera gf_camera(const Object *camera_obj,
                        const RenderData *rd,
                        pxr::GfVec2i res,
                        pxr::GfVec4f tile)
{
  const Camera *camera = (const Camera *)camera_obj->data;

  CameraParams camera_params;
  BKE_camera_params_init(&camera_params);
  BKE_camera_params_from_object(&camera_params, camera_obj);

  pxr::GfMatrix4d transform = gf_matrix_from_transform(camera_obj->object_to_world);
  pxr::GfRange1f clip_range = pxr::GfRange1f(camera_params.clip_start, camera_params.clip_end);

  float ratio = float(res[0]) / res[1];

  int mode;
  pxr::GfVec2f lens_shift;
  pxr::GfVec2f aperture;
  int sensor_fit = BKE_camera_sensor_fit(
      camera_params.sensor_fit, rd->xasp * rd->xsch, rd->yasp * rd->ysch);

  switch (sensor_fit) {
    case CAMERA_SENSOR_FIT_VERT:
      lens_shift = pxr::GfVec2f(camera_params.shiftx / ratio, camera_params.shifty);
      break;
    case CAMERA_SENSOR_FIT_HOR:
      lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty * ratio);
      break;
    case CAMERA_SENSOR_FIT_AUTO:
      if (ratio > 1.0f) {
        lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty * ratio);
      }
      else {
        lens_shift = pxr::GfVec2f(camera_params.shiftx / ratio, camera_params.shifty);
      }
      break;
    default:
      BLI_assert_unreachable();
  }

  float t_pos[2] = {tile[0], tile[1]};
  float t_size[2] = {tile[2], tile[3]};
  lens_shift = pxr::GfVec2f(
      lens_shift[0] / t_size[0] + (t_pos[0] + t_size[0] * 0.5 - 0.5) / t_size[0],
      lens_shift[1] / t_size[1] + (t_pos[1] + t_size[1] * 0.5 - 0.5) / t_size[1]);

  switch (camera->type) {
    case CAM_PANO:
    case CAM_PERSP:
      mode = camera->type == CAM_PERSP ? CAM_PERSP : CAM_PANO;
      switch (sensor_fit) {
        case CAMERA_SENSOR_FIT_VERT:
          aperture = pxr::GfVec2f(camera_params.sensor_y * ratio, camera_params.sensor_y);
          break;
        case CAMERA_SENSOR_FIT_HOR:
          aperture = pxr::GfVec2f(camera_params.sensor_x, camera_params.sensor_x / ratio);
          break;
        case CAMERA_SENSOR_FIT_AUTO:
          if (ratio > 1.0f) {
            aperture = pxr::GfVec2f(camera_params.sensor_x, camera_params.sensor_x / ratio);
          }
          else {
            aperture = pxr::GfVec2f(camera_params.sensor_x * ratio, camera_params.sensor_x);
          }
          break;
        default:
          BLI_assert_unreachable();
      }
      aperture = pxr::GfVec2f(aperture[0] * t_size[0], aperture[1] * t_size[1]);
      break;
    case CAM_ORTHO:
      mode = CAM_ORTHO;
      switch (sensor_fit) {
        case CAMERA_SENSOR_FIT_VERT:
          aperture = pxr::GfVec2f(camera_params.ortho_scale * ratio, camera_params.ortho_scale);
          break;
        case CAMERA_SENSOR_FIT_HOR:
          aperture = pxr::GfVec2f(camera_params.ortho_scale, camera_params.ortho_scale / ratio);
          break;
        case CAMERA_SENSOR_FIT_AUTO:
          if (ratio > 1.0f) {
            aperture = pxr::GfVec2f(camera_params.ortho_scale, camera_params.ortho_scale / ratio);
          }
          else {
            aperture = pxr::GfVec2f(camera_params.ortho_scale * ratio, camera_params.ortho_scale);
          }
          break;
        default:
          BLI_assert_unreachable();
      }
      aperture = pxr::GfVec2f(aperture[0] * t_size[0] * 10, aperture[1] * t_size[1] * 10);
      break;
    default:
      BLI_assert_unreachable();
  }
  return set_gf_camera(camera_params.is_ortho, transform, aperture, lens_shift, clip_range);
}

}  // namespace blender::io::hydra
