/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <sstream>
#include <filesystem>

#include <pxr/base/tf/stringUtils.h>

#include "BKE_appdir.h"
#include "BKE_image_save.h"
#include "BLI_string.h"
#include "BLI_path_util.h"

#include "DNA_camera_types.h"

#include "utils.h"

using namespace std;
using namespace pxr;

namespace blender::render::hydra {

GfMatrix4d gf_matrix_from_transform(float m[4][4])
{
  return GfMatrix4d(
    m[0][0], m[0][1], m[0][2], m[0][3],
    m[1][0], m[1][1], m[1][2], m[1][3],
    m[2][0], m[2][1], m[2][2], m[2][3],
    m[3][0], m[3][1], m[3][2], m[3][3]);
}

string format_duration(chrono::milliseconds millisecs)
{
  stringstream ss;
  bool neg = millisecs < 0ms;
  if (neg) {
    millisecs = -millisecs;
  }
  auto m = chrono::duration_cast<chrono::minutes>(millisecs);
  millisecs -= m;
  auto s = chrono::duration_cast<chrono::seconds>(millisecs);
  millisecs -= s;
  if (neg) {
    ss << "-";
  }
  if (m < 10min) {
    ss << "0";
  }
  ss << to_string(m / 1min) << ":";
  if (s < 10s) {
    ss << "0";
  }
  ss << to_string(s / 1s) << ":";
  if (millisecs < 10ms) {
    ss << "0";
  }
  ss << to_string(millisecs / 1ms / 10);
  return ss.str();
}

string cache_image(Main *bmain,
                   Scene *scene,
                   Image *image,
                   ImageUser *iuser,
                   ImageSaveOptions *opts,
                   ReportList *reports)
{
  const string default_format = ".png";

  char tempfile[FILE_MAX];

  if (!BKE_image_save_options_init(opts, bmain, scene, image, iuser, true, false)) {
    BKE_image_save_options_free(opts);
    return "";
  }

  string image_name;

  if (image->source == IMA_SRC_GENERATED) {
    image_name = TfMakeValidIdentifier(image_name.append(image->id.name + 2));
  }
  else {
    image_name = image->filepath == NULL ? image->filepath : image->id.name + 2;
    image_name = std::filesystem::path(image_name).filename().replace_extension().string();
    image_name = TfMakeValidIdentifier(image_name);
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

}  // namespace blender::render::hydra
