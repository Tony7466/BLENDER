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

CameraData::CameraData(const Depsgraph *depsgraph,
                       const View3D *v3d,
                       const ARegion *region,
                       RenderData *rd)
{
  const RegionView3D *region_data = (const RegionView3D *)region->regiondata;

  BKE_camera_params_init(&camera_params_);
  BKE_camera_params_from_view3d(&camera_params_, depsgraph, v3d, region_data);

  int sensor_size = BKE_camera_sensor_size(
      camera_params_.sensor_fit, camera_params_.sensor_x, camera_params_.sensor_y);
  sensor_size *= 2.0f;

  pxr::GfVec2i res(region->winx, region->winy);
  float ratio = float(res[0]) / res[1];
  transform_ = gf_matrix_from_transform(region_data->viewmat).GetInverse();

  switch (region_data->persp) {
    case RV3D_PERSP:
      mode_ = CAM_PERSP;
      clip_range_ = pxr::GfRange1f(camera_params_.clip_start, camera_params_.clip_end);
      lens_shift_ = pxr::GfVec2f(camera_params_.shiftx, camera_params_.shifty);

      if (ratio > 1.0f) {
        sensor_size_ = pxr::GfVec2f(sensor_size, sensor_size / ratio);
      }
      else {
        sensor_size_ = pxr::GfVec2f(sensor_size * ratio, sensor_size);
      }
      break;
    case RV3D_ORTHO: {
      mode_ = CAM_ORTHO;
      lens_shift_ = pxr::GfVec2f(camera_params_.shiftx, camera_params_.shifty);

      float o_size = region_data->dist * sensor_size / camera_params_.lens;
      float o_depth = v3d->clip_end;

      clip_range_ = pxr::GfRange1f(-o_depth * 0.5, o_depth * 0.5);

      if (ratio > 1.0f) {
        ortho_size_ = pxr::GfVec2f(o_size, o_size / ratio);
      }
      else {
        ortho_size_ = pxr::GfVec2f(o_size * ratio, o_size);
      }
      break;
    }

    case RV3D_CAMOB: {
      pxr::GfMatrix4d mat = transform_;
      *this = CameraData(v3d->camera, res, pxr::GfVec4f(0, 0, 1, 1), rd);
      transform_ = mat;

      /* This formula was taken from previous plugin with corresponded comment.
       * See blender/intern/cycles/blender/blender_camera.cpp:blender_camera_from_view (look
       * for 1.41421f). */
      float zoom = 4.0 / pow((pow(2.0, 0.5) + region_data->camzoom / 50.0), 2);

      /* Updating l_shift due to viewport zoom and view_camera_offset
       * view_camera_offset should be multiplied by 2. */
      lens_shift_ = pxr::GfVec2f((lens_shift_[0] + camera_params_.offsetx * 2) / zoom,
                                 (lens_shift_[1] + camera_params_.offsety * 2) / zoom);

      if (camera_params_.is_ortho) {
        ortho_size_ *= zoom;
      }
      else {
        sensor_size_ *= zoom;
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }
}

CameraData::CameraData(const Object *camera_obj,
                       pxr::GfVec2i res,
                       pxr::GfVec4f tile,
                       RenderData *rd)
{
  const Camera *camera = (const Camera *)camera_obj->data;

  BKE_camera_params_init(&camera_params_);
  BKE_camera_params_from_object(&camera_params_, camera_obj);

  float t_pos[2] = {tile[0], tile[1]};
  float t_size[2] = {tile[2], tile[3]};
  transform_ = gf_matrix_from_transform(camera_obj->object_to_world);
  clip_range_ = pxr::GfRange1f(camera_params_.clip_start, camera_params_.clip_end);

  if (camera->dof.flag & CAM_DOF_ENABLED) {
    dof_data_ = std::tuple(BKE_camera_object_dof_distance(camera_obj),
                           camera->dof.aperture_fstop,
                           camera->dof.aperture_blades);
  }

  float ratio = float(res[0]) / res[1];
  int sensor_fit = BKE_camera_sensor_fit(
      camera_params_.sensor_fit, rd->xasp * rd->xsch, rd->yasp * rd->ysch);

  switch (sensor_fit) {
    case CAMERA_SENSOR_FIT_VERT:
      lens_shift_ = pxr::GfVec2f(camera_params_.shiftx / ratio, camera_params_.shifty);
      break;
    case CAMERA_SENSOR_FIT_HOR:
      lens_shift_ = pxr::GfVec2f(camera_params_.shiftx, camera_params_.shifty * ratio);
      break;
    case CAMERA_SENSOR_FIT_AUTO:
      if (ratio > 1.0f) {
        lens_shift_ = pxr::GfVec2f(camera_params_.shiftx, camera_params_.shifty * ratio);
      }
      else {
        lens_shift_ = pxr::GfVec2f(camera_params_.shiftx / ratio, camera_params_.shifty);
      }
      break;
    default:
      BLI_assert_unreachable();
  }

  lens_shift_ = pxr::GfVec2f(
      lens_shift_[0] / t_size[0] + (t_pos[0] + t_size[0] * 0.5 - 0.5) / t_size[0],
      lens_shift_[1] / t_size[1] + (t_pos[1] + t_size[1] * 0.5 - 0.5) / t_size[1]);

  switch (camera->type) {
    case CAM_PANO:
    case CAM_PERSP:
      mode_ = camera->type == CAM_PERSP ? CAM_PERSP : CAM_PANO;
      switch (sensor_fit) {
        case CAMERA_SENSOR_FIT_VERT:
          sensor_size_ = pxr::GfVec2f(camera_params_.sensor_y * ratio, camera_params_.sensor_y);
          break;
        case CAMERA_SENSOR_FIT_HOR:
          sensor_size_ = pxr::GfVec2f(camera_params_.sensor_x, camera_params_.sensor_x / ratio);
          break;
        case CAMERA_SENSOR_FIT_AUTO:
          if (ratio > 1.0f) {
            sensor_size_ = pxr::GfVec2f(camera_params_.sensor_x, camera_params_.sensor_x / ratio);
          }
          else {
            sensor_size_ = pxr::GfVec2f(camera_params_.sensor_x * ratio, camera_params_.sensor_x);
          }
          break;
        default:
          BLI_assert_unreachable();
      }
      sensor_size_ = pxr::GfVec2f(sensor_size_[0] * t_size[0], sensor_size_[1] * t_size[1]);
      break;
    case CAM_ORTHO:
      mode_ = CAM_ORTHO;
      switch (sensor_fit) {
        case CAMERA_SENSOR_FIT_VERT:
          ortho_size_ = pxr::GfVec2f(camera_params_.ortho_scale * ratio,
                                     camera_params_.ortho_scale);
          break;
        case CAMERA_SENSOR_FIT_HOR:
          ortho_size_ = pxr::GfVec2f(camera_params_.ortho_scale,
                                     camera_params_.ortho_scale / ratio);
          break;
        case CAMERA_SENSOR_FIT_AUTO:
          if (ratio > 1.0f) {
            ortho_size_ = pxr::GfVec2f(camera_params_.ortho_scale,
                                       camera_params_.ortho_scale / ratio);
          }
          else {
            ortho_size_ = pxr::GfVec2f(camera_params_.ortho_scale * ratio,
                                       camera_params_.ortho_scale);
          }
          break;
        default:
          BLI_assert_unreachable();
      }
      ortho_size_ = pxr::GfVec2f(ortho_size_[0] * t_size[0], ortho_size_[1] * t_size[1]);
      break;
    default:
      BLI_assert_unreachable();
  }
}

pxr::GfCamera CameraData::gf_camera()
{
  return gf_camera(pxr::GfVec4f(0, 0, 1, 1));
}

pxr::GfCamera CameraData::gf_camera(pxr::GfVec4f tile)
{
  if (mode_ == CAM_PANO) {
    CLOG_WARN(LOG_HYDRA_SCENE, "Unsupported camera type: %d, perspective will be used", mode_);
  }
  float t_pos[2] = {tile[0], tile[1]}, t_size[2] = {tile[2], tile[3]};

  pxr::GfCamera gf_camera = pxr::GfCamera();

  gf_camera.SetClippingRange(clip_range_);

  float l_shift[2] = {(lens_shift_[0] + t_pos[0] + t_size[0] * 0.5f - 0.5f) / t_size[0],
                      (lens_shift_[1] + t_pos[1] + t_size[1] * 0.5f - 0.5f) / t_size[1]};

  if (camera_params_.is_ortho) {
    gf_camera.SetProjection(pxr::GfCamera::Projection::Orthographic);

    /* Use tenths of a world unit according to USD docs
     * https://graphics.pixar.com/usd/docs/api/class_gf_camera.html */
    float o_size[2] = {ortho_size_[0] * t_size[0] * 10, ortho_size_[1] * t_size[1] * 10};

    gf_camera.SetHorizontalAperture(o_size[0]);
    gf_camera.SetVerticalAperture(o_size[1]);

    gf_camera.SetHorizontalApertureOffset(l_shift[0] * o_size[0]);
    gf_camera.SetVerticalApertureOffset(l_shift[1] * o_size[1]);
  }
  else {
    gf_camera.SetProjection(pxr::GfCamera::Projection::Perspective);

    float s_size[2] = {sensor_size_[0] * t_size[0], sensor_size_[1] * t_size[1]};

    gf_camera.SetHorizontalAperture(s_size[0]);
    gf_camera.SetVerticalAperture(s_size[1]);

    gf_camera.SetHorizontalApertureOffset(l_shift[0] * s_size[0]);
    gf_camera.SetVerticalApertureOffset(l_shift[1] * s_size[1]);
  }

  gf_camera.SetTransform(transform_);
  return gf_camera;
}

}  // namespace blender::io::hydra
