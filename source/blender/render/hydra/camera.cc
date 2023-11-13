/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "camera.h"

#include "BKE_camera.h"

#include "DNA_camera_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"

#include "hydra/object.h"

namespace blender::render::hydra {

static pxr::GfCamera create_gf_camera(bool is_ortho,
                                      const pxr::GfMatrix4d &transform,
                                      const pxr::GfVec2f &aperture,
                                      const pxr::GfVec2f &lens_shift,
                                      const pxr::GfRange1f &clip_range,
                                      const float &focal_length)
{
  pxr::GfCamera gf_camera = pxr::GfCamera();

  gf_camera.SetProjection(is_ortho ? pxr::GfCamera::Projection::Orthographic :
                                     pxr::GfCamera::Projection::Perspective);
  gf_camera.SetHorizontalAperture(aperture[0]);
  gf_camera.SetVerticalAperture(aperture[1]);
  gf_camera.SetHorizontalApertureOffset(lens_shift[0] * aperture[0]);
  gf_camera.SetVerticalApertureOffset(lens_shift[1] * aperture[1]);
  gf_camera.SetClippingRange(clip_range);
  gf_camera.SetTransform(transform);
  gf_camera.SetFocalLength(focal_length);

  return gf_camera;
}

pxr::GfCamera gf_camera(const Depsgraph *depsgraph,
                        const View3D *v3d,
                        const ARegion *region,
                        const RenderData *rd,
                        const pxr::GfVec4f &border)
{
  CameraParams camera_params;
  const RegionView3D *region_data = (const RegionView3D *)region->regiondata;

  BKE_camera_params_init(&camera_params);
  BKE_camera_params_from_view3d(&camera_params, depsgraph, v3d, region_data);

  pxr::GfRange1f clip_range;
  pxr::GfVec2f lens_shift;
  pxr::GfVec2f sensor_size;
  pxr::GfVec2f ortho_size;
  pxr::GfMatrix4d transform =
      io::hydra::gf_matrix_from_transform(region_data->viewmat).GetInverse();
  clip_range = pxr::GfRange1f(camera_params.clip_start, camera_params.clip_end);

  const float fit_width = (region_data->persp == RV3D_CAMOB) ? rd->xasp * rd->xsch : region->winx;
  const float fit_height = (region_data->persp == RV3D_CAMOB) ? rd->yasp * rd->ysch : region->winy;
  const int sensor_fit = BKE_camera_sensor_fit(
      camera_params.sensor_fit, fit_width, fit_height);
  float ratio = float(region->winx) / region->winy;
  const pxr::GfVec2f sensor_fit_scale = (sensor_fit == CAMERA_SENSOR_FIT_HOR) ?
                                            pxr::GfVec2f(1.0f, 1.0f / ratio) :
                                            pxr::GfVec2f(1.0f * ratio, 1.0f);

  lens_shift = pxr::GfVec2f(camera_params.shiftx, camera_params.shifty);
  lens_shift = pxr::GfCompDiv(lens_shift, sensor_fit_scale);
  lens_shift += pxr::GfVec2f(camera_params.offsetx, camera_params.offsety) * 2;

  pxr::GfVec2f b_pos(border[0], border[1]), b_size(border[2], border[3]);
  lens_shift += b_pos + b_size * 0.5f - pxr::GfVec2f(0.5f);
  lens_shift = pxr::GfCompDiv(lens_shift, b_size);

  float camera_sensor_size;
  if (region_data->persp == RV3D_CAMOB) {
    camera_sensor_size = BKE_camera_sensor_size(
        sensor_fit, camera_params.sensor_x, camera_params.sensor_y);
  }
  else {
    camera_sensor_size = BKE_camera_sensor_size(
        camera_params.sensor_fit, camera_params.sensor_x, camera_params.sensor_y);
  }

  
  pxr::GfVec2f aperture = pxr::GfVec2f((camera_params.is_ortho) ? camera_params.ortho_scale :
                                                                  camera_sensor_size);
  aperture = pxr::GfCompMult(aperture, sensor_fit_scale);
  aperture *= camera_params.zoom;
  if (camera_params.is_ortho) {
    /* Use tenths of a world unit according to USD docs
     * https://graphics.pixar.com/usd/docs/api/class_gf_camera.html */
    aperture = pxr::GfCompMult(aperture, b_size) * 10;
  }
  else {
    aperture = pxr::GfCompMult(aperture, b_size);
  }
  return create_gf_camera(
      camera_params.is_ortho, transform, aperture, lens_shift, clip_range, camera_params.lens);
}

pxr::GfCamera gf_camera(const Object *camera_obj,
                        const RenderData *rd,
                        const pxr::GfVec2i &res,
                        const pxr::GfVec4f &border)
{
  const Camera *camera = (const Camera *)camera_obj->data;

  CameraParams camera_params;
  BKE_camera_params_init(&camera_params);
  BKE_camera_params_from_object(&camera_params, camera_obj);

  pxr::GfMatrix4d transform = io::hydra::gf_matrix_from_transform(camera_obj->object_to_world);
  pxr::GfRange1f clip_range = pxr::GfRange1f(camera_params.clip_start, camera_params.clip_end);

  float ratio = float(res[0]) / res[1];

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

  pxr::GfVec2f b_pos(border[0], border[1]), b_size(border[2], border[3]);
  lens_shift += b_pos + b_size * 0.5f - pxr::GfVec2f(0.5f);
  lens_shift = pxr::GfCompDiv(lens_shift, b_size);

  switch (camera->type) {
    case CAM_PANO:
    case CAM_PERSP:
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
      aperture = pxr::GfCompMult(aperture, b_size);
      break;

    case CAM_ORTHO:
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
      aperture = pxr::GfCompMult(aperture, b_size) * 10;
      break;

    default:
      BLI_assert_unreachable();
  }
  return create_gf_camera(
      camera_params.is_ortho, transform, aperture, lens_shift, clip_range, camera_params.lens);
}

}  // namespace blender::render::hydra
