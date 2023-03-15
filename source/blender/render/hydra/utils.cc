/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <chrono>
#include <filesystem>
#include <sstream>

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

std::string format_duration(std::chrono::milliseconds millisecs)
{
  std::stringstream ss;
  bool neg = millisecs < std::chrono::milliseconds(0);
  if (neg) {
    millisecs = -millisecs;
  }
  auto m = std::chrono::duration_cast<std::chrono::minutes>(millisecs);
  millisecs -= m;
  auto s = std::chrono::duration_cast<std::chrono::seconds>(millisecs);
  millisecs -= s;
  if (neg) {
    ss << "-";
  }
  if (m < std::chrono::minutes(10)) {
    ss << "0";
  }
  ss << std::to_string(m / std::chrono::minutes(1)) << ":";
  if (s < std::chrono::seconds(10)) {
    ss << "0";
  }
  ss << std::to_string(s / std::chrono::seconds(1)) << ":";
  if (millisecs < std::chrono::milliseconds(10)) {
    ss << "0";
  }
  ss << std::to_string(millisecs / std::chrono::milliseconds(1) / 10);
  return ss.str();
}

std::string cache_image(Main *bmain,
                        Scene *scene,
                        Image *image,
                        ImageUser *iuser,
                        ImageSaveOptions *opts,
                        ReportList *reports)
{
  const std::string default_format = ".png";

  char tempfile[FILE_MAX];

  if (!BKE_image_save_options_init(opts, bmain, scene, image, iuser, true, false)) {
    BKE_image_save_options_free(opts);
    return "";
  }

  std::string image_name;

  if (image->source == IMA_SRC_GENERATED) {
    image_name = pxr::TfMakeValidIdentifier(image_name.append(image->id.name + 2));
  }
  else {
    image_name = image->filepath == NULL ? image->filepath : image->id.name + 2;
    image_name = std::filesystem::path(image_name).filename().replace_extension().string();
    image_name = pxr::TfMakeValidIdentifier(image_name);
  }

  image_name.append(default_format);

  BLI_path_join(tempfile, sizeof(tempfile), BKE_tempdir_session(), image_name.c_str());
  STRNCPY(opts->filepath, tempfile);

  if (!BKE_image_save(reports, bmain, image, iuser, opts)) {
    BKE_image_save_options_free(opts);
    return "";
  };

  BKE_image_save_options_free(opts);
  return tempfile;
}

void set_env_paths(std::string const &name, std::vector<std::string> path_dirs)
{
  const char *env = BLI_getenv(name.c_str());
  ;
  std::stringstream ss;
  int i = 0;
  for (std::string &s : path_dirs) {
    ++i;
    ss << s;
    if (i < path_dirs.size() || env) {
#ifdef _WIN32
      ss << ";";
#else
      ss << ":";
#endif
    }
  }
  if (env) {
    ss << env;
  }
  BLI_setenv(name.c_str(), ss.str().c_str());
}

}  // namespace blender::render::hydra
