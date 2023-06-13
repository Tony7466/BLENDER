/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hio/imageRegistry.h>

#include "BKE_appdir.h"
#include "BKE_image_format.h"
#include "BKE_image_save.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"

#include "blender_scene_delegate.h"
#include "image.h"

namespace blender::render::hydra {

static std::string cache_image_file(Image *image,
                                    bContext *context,
                                    ImageUser *iuser,
                                    bool check_exist)
{
  char file_path[FILE_MAX];
  Main *main = CTX_data_main(context);
  Scene *scene = CTX_data_scene(context);
  ImageSaveOptions opts;
  if (BKE_image_save_options_init(&opts, main, scene, image, iuser, false, false)) {
    char file_name[32];
    const char *r_ext;
    BKE_image_path_ext_from_imformat(&scene->r.im_format, &r_ext);
    snprintf(file_name, sizeof(file_name), "img_%016llx%s", (uintptr_t)image, r_ext);

    BLI_path_join(
        file_path, sizeof(file_path), BKE_tempdir_session(), "hydra_image_cache", file_name);

    if (check_exist && BLI_exists(file_path)) {
      return file_path;
    }
    opts.save_copy = true;
    STRNCPY(opts.filepath, file_path);
    if (BKE_image_save(nullptr, main, image, iuser, &opts)) {
      CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path);
    }
    else {
      memset(file_path, 0, sizeof(file_path));
    }
  }
  BKE_image_save_options_free(&opts);

  return file_path;
}

std::string cache_or_get_image_file(Image *image, bContext *context, ImageUser *iuser)
{
  std::string file_path(FILE_MAX, 0);
  if (image->source == IMA_SRC_GENERATED) {
    file_path = cache_image_file(image, context, iuser, false);
  }
  else if (BKE_image_has_packedfile(image)) {
    file_path = cache_image_file(image, context, iuser, true);
  }
  else {
    Main *main = CTX_data_main(context);
    BKE_image_user_file_path_ex(main, iuser, image, file_path.data(), false, true);

    if (!pxr::HioImageRegistry::GetInstance().IsSupportedImageFile(file_path)) {
      file_path = cache_image_file(image, context, iuser, true);
    }
  }

  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path.c_str());
  return file_path;
}

}  // namespace blender::render::hydra
