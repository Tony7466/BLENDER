/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <filesystem>

#include <pxr/base/tf/stringUtils.h>

#include "BKE_appdir.h"
#include "BKE_image_save.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "DNA_camera_types.h"

#include "utils.h"

namespace blender::render::hydra {

pxr::GfMatrix4d gf_matrix_from_transform(float m[4][4])
{
  pxr::GfMatrix4d ret = pxr::GfMatrix4d();
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ret[i][j] = m[i][j];
    }
  }
  return ret;
}

std::string cache_image(Main *bmain,
                        Scene *scene,
                        Image *image,
                        ImageUser *iuser,
                        ImageSaveOptions *opts,
                        ReportList *reports)
{
  const char *default_format = ".png";
  char tempfile[FILE_MAX];

  if (!BKE_image_save_options_init(opts, bmain, scene, image, iuser, true, false)) {
    BKE_image_save_options_free(opts);
    return "";
  }

  char image_name[32];
  snprintf(image_name, 32, "img_%016llx", (uint64_t)image);

  strcat(image_name, default_format);

  BLI_path_join(
      tempfile, sizeof(tempfile), BKE_tempdir_session(), "hydra_image_cache", image_name);
  STRNCPY(opts->filepath, tempfile);

  if (!BKE_image_save(reports, bmain, image, iuser, opts)) {
    BKE_image_save_options_free(opts);
    return "";
  };

  BKE_image_save_options_free(opts);
  return tempfile;
}

}  // namespace blender::render::hydra
