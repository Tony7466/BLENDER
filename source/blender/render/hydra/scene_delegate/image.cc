/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hio/imageRegistry.h>

#include "BKE_appdir.h"
#include "BKE_image.h"
#include "BKE_image_save.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"

#include "DNA_windowmanager_types.h"

#include "blender_scene_delegate.h"
#include "image.h"

namespace blender::render::hydra {

static std::string cache_image_file(Image *image,
                                    BlenderSceneDelegate *scene_delegate,
                                    ImageUser *iuser,
                                    bool check_exist)
{
  std::string file_path(FILE_MAX, 0);
  char file_name[32];
  snprintf(file_name, 32, "img_%016llx.hdr", (uintptr_t)image);
  BLI_path_join(file_path.data(),
                file_path.capacity(),
                BKE_tempdir_session(),
                "hydra_image_cache",
                file_name);

  if (check_exist && BLI_exists(file_path.c_str())) {
    return file_path;
  }

  Main *main = CTX_data_main(scene_delegate->context);
  ImageSaveOptions opts;
  opts.im_format.imtype = R_IMF_IMTYPE_RADHDR;

  if (BKE_image_save_options_init(&opts, main, scene_delegate->scene, image, iuser, true, false)) {
    STRNCPY(opts.filepath, file_path.c_str());
    ReportList reports;
    if (BKE_image_save(&reports, main, image, iuser, &opts)) {
      CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path.c_str());
    }
    else {
      file_path = "";
    }
  }
  BKE_image_save_options_free(&opts);

  return file_path;
}

std::string cache_or_get_image_file(Image *image,
                                    BlenderSceneDelegate *scene_delegate,
                                    ImageUser *iuser)
{
  std::string file_path(FILE_MAX, 0);
  if (image->source == IMA_SRC_GENERATED) {
    file_path = cache_image_file(image, scene_delegate, iuser, false);
  }
  else if (BKE_image_has_packedfile(image)) {
    file_path = cache_image_file(image, scene_delegate, iuser, true);
  }
  else {
    Main *main = CTX_data_main(scene_delegate->context);
    BKE_image_user_file_path_ex(main, iuser, image, file_path.data(), false, true);

    if (!pxr::HioImageRegistry::GetInstance().IsSupportedImageFile(file_path)) {
      file_path = cache_image_file(image, scene_delegate, iuser, true);
    }
  }

  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path.c_str());
  return file_path;
}

}  // namespace blender::render::hydra
