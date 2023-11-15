/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "image.h"

#include <pxr/imaging/hio/imageRegistry.h>

#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BKE_appdir.h"
#include "BKE_image.h"
#include "BKE_image_format.h"
#include "BKE_image_save.h"
#include "BKE_main.h"
#include "BKE_packedFile.h"

#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"

#include "hydra_scene_delegate.h"

namespace blender::io::hydra {

std::string image_cache_file_path()
{
  char dir_path[FILE_MAX];
  BLI_path_join(dir_path, sizeof(dir_path), BKE_tempdir_session(), "hydra", "image_cache");
  return dir_path;
}

static std::string get_cache_file(const std::string &file_name, bool mkdir = true)
{
  std::string dir_path = image_cache_file_path();
  if (mkdir) {
    BLI_dir_create_recursive(dir_path.c_str());
  }

  char file_path[FILE_MAX];
  BLI_path_join(file_path, sizeof(file_path), dir_path.c_str(), file_name.c_str());
  return file_path;
}

static std::string cache_image_file(
    Main *bmain, Scene *scene, Image *image, ImageUser *iuser, bool check_exist)
{
  std::string file_path;
  ImageSaveOptions opts;
  if (BKE_image_save_options_init(&opts, bmain, scene, image, iuser, false, false)) {
    char file_name[32];
    const char *r_ext = BLI_path_extension_or_end(image->id.name);
    if (!pxr::HioImageRegistry::GetInstance().IsSupportedImageFile(image->id.name)) {
      BKE_image_path_ext_from_imformat(&scene->r.im_format, &r_ext);
      opts.im_format = scene->r.im_format;
    }

    SNPRINTF(file_name, "img_%p%s", image, r_ext);

    file_path = get_cache_file(file_name);
    if (check_exist && BLI_exists(file_path.c_str())) {
      return file_path;
    }

    opts.save_copy = true;
    STRNCPY(opts.filepath, file_path.c_str());
    if (BKE_image_save(nullptr, bmain, image, iuser, &opts)) {
      CLOG_INFO(LOG_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path.c_str());
    }
    else {
      CLOG_ERROR(LOG_HYDRA_SCENE, "Can't save %s", file_path.c_str());
      file_path = "";
    }
  }
  BKE_image_save_options_free(&opts);
  return file_path;
}

std::string cache_or_get_image_file(Main *bmain, Scene *scene, Image *image, ImageUser *iuser)
{
  char str[FILE_MAX];
  std::string file_path;
  bool do_check_extension = false;
  if (image->source == IMA_SRC_GENERATED) {
    file_path = cache_image_file(bmain, scene, image, iuser, false);
  }
  else if (BKE_image_has_packedfile(image)) {
    do_check_extension = true;
    std::string dir_path = image_cache_file_path();
    char *cached_path;
    char subfolder[FILE_MAXDIR];
    SNPRINTF(subfolder, "unpack_%p", image);
    LISTBASE_FOREACH (ImagePackedFile *, ipf, &image->packedfiles) {
      char path[FILE_MAX];
      BLI_path_join(
          path, sizeof(path), dir_path.c_str(), subfolder, BLI_path_basename(ipf->filepath));
      cached_path = BKE_packedfile_unpack_to_file(nullptr,
                                                  BKE_main_blendfile_path(bmain),
                                                  dir_path.c_str(),
                                                  path,
                                                  ipf->packedfile,
                                                  PF_WRITE_LOCAL);

      /* Take first successfully unpacked image. */
      if (cached_path != nullptr) {
        if (file_path.empty()) {
          file_path = cached_path;
        }
        MEM_freeN(cached_path);
      }
    }
  }
  else {
    do_check_extension = true;
    BKE_image_user_file_path_ex(bmain, iuser, image, str, false, true);
    file_path = str;
  }

  if (do_check_extension && !pxr::HioImageRegistry::GetInstance().IsSupportedImageFile(file_path))
  {
    file_path = cache_image_file(bmain, scene, image, iuser, true);
  }

  CLOG_INFO(LOG_HYDRA_SCENE, 1, "%s -> %s", image->id.name, file_path.c_str());
  return file_path;
}

std::string cache_image_color(float color[4])
{
  char name[128];
  SNPRINTF(name,
           "color_%02d%02d%02d.hdr",
           int(color[0] * 255),
           int(color[1] * 255),
           int(color[2] * 255));
  std::string file_path = get_cache_file(name);
  if (BLI_exists(file_path.c_str())) {
    return file_path;
  }

  ImBuf *ibuf = IMB_allocImBuf(4, 4, 32, IB_rectfloat);
  IMB_rectfill(ibuf, color);
  ibuf->ftype = IMB_FTYPE_RADHDR;

  if (IMB_saveiff(ibuf, file_path.c_str(), IB_rectfloat)) {
    CLOG_INFO(LOG_HYDRA_SCENE, 1, "%s", file_path.c_str());
  }
  else {
    CLOG_ERROR(LOG_HYDRA_SCENE, "Can't save %s", file_path.c_str());
    file_path = "";
  }
  IMB_freeImBuf(ibuf);

  return file_path;
}

/* Get the absolute filepath of the given image.  Assumes
 * r_path result array is of length FILE_MAX. */
static void get_absolute_path(Image *ima, char *r_path)
{
  /* Make absolute source path. */
  BLI_strncpy(r_path, ima->filepath, FILE_MAX);
  BLI_path_abs(r_path, ID_BLEND_PATH_FROM_GLOBAL(&ima->id));
  BLI_path_normalize(r_path);
}

/* Copy the given image to the destination directory. */
static std::string copy_single_file(Image *ima,
                                    const std::string &dest_dir,
                                    const bool allow_overwrite)
{
  char source_path[FILE_MAX];
  get_absolute_path(ima, source_path);

  char file_name[FILE_MAX];
  BLI_path_split_file_part(source_path, file_name, FILE_MAX);

  char dest_path[FILE_MAX];
  BLI_path_join(dest_path, FILE_MAX, dest_dir.c_str(), file_name);

  if (!allow_overwrite && BLI_exists(dest_path)) {
    return "";
  }

  if (BLI_path_cmp_normalized(source_path, dest_path) == 0) {
    /* Source and destination paths are the same, don't copy. */
    return "";
  }

  CLOG_INFO(LOG_HYDRA_SCENE, 1, "Copying texture from %s to %s", source_path, dest_path);

  /* Copy the file. */
  if (BLI_copy(source_path, dest_path) != 0) {
    CLOG_WARN(LOG_HYDRA_SCENE,
              "USD export: could not copy texture from %s to %s",
              source_path,
              dest_path);
    return "";
  }

  return dest_path;
}

static std::string export_in_memory_texture(Image *ima,
                                            const std::string &export_dir,
                                            const bool allow_overwrite)
{
  char image_abs_path[FILE_MAX];

  char file_name[FILE_MAX];
  if (strlen(ima->filepath) > 0) {
    get_absolute_path(ima, image_abs_path);
    BLI_path_split_file_part(image_abs_path, file_name, FILE_MAX);
  }
  else {
    /* Use the image name for the file name. */
    STRNCPY(file_name, ima->id.name + 2);
  }

  ImBuf *imbuf = BKE_image_acquire_ibuf(ima, nullptr, nullptr);
  BLI_SCOPED_DEFER([&]() { BKE_image_release_ibuf(ima, imbuf, nullptr); });
  if (!imbuf) {
    return "";
  }

  ImageFormatData imageFormat;
  BKE_image_format_from_imbuf(&imageFormat, imbuf);

  /* This image in its current state only exists in Blender memory.
   * So we have to export it. The export will keep the image state intact,
   * so the exported file will not be associated with the image. */

  BKE_image_path_ext_from_imformat_ensure(file_name, sizeof(file_name), &imageFormat);

  char export_path[FILE_MAX];
  BLI_path_join(export_path, FILE_MAX, export_dir.c_str(), file_name);

  if (!allow_overwrite && BLI_exists(export_path)) {
    return "";
  }

  if ((BLI_path_cmp_normalized(export_path, image_abs_path) == 0) && BLI_exists(image_abs_path)) {
    /* As a precaution, don't overwrite the original path. */
    return image_abs_path;
  }

  CLOG_INFO(LOG_HYDRA_SCENE, 1, "Exporting in-memory texture to %s", export_path);

  if (BKE_imbuf_write_as(imbuf, export_path, &imageFormat, true) == 0) {
    CLOG_WARN(LOG_HYDRA_SCENE, "USD export: couldn't export in-memory texture to %s", export_path);
  }

  return image_abs_path;
}

/* If the given image is tiled, copy the image tiles to the given
 * destination directory. */
static std::string copy_tiled_textures(Image *ima,
                                       const std::string &dest_dir,
                                       const bool allow_overwrite)
{
  char src_path[FILE_MAX];
  get_absolute_path(ima, src_path);

  eUDIM_TILE_FORMAT tile_format;
  char *udim_pattern = BKE_image_get_tile_strformat(src_path, &tile_format);

  /* Only <UDIM> tile formats are supported by USD right now. */
  if (tile_format != UDIM_TILE_FORMAT_UDIM) {
    CLOG_WARN(LOG_HYDRA_SCENE, "Unsupported tile format for `%s`", src_path);
    MEM_SAFE_FREE(udim_pattern);
    return "";
  }

  /* Copy all tiles. */
  LISTBASE_FOREACH (ImageTile *, tile, &ima->tiles) {
    char src_tile_path[FILE_MAX];
    BKE_image_set_filepath_from_tile_number(
        src_tile_path, udim_pattern, tile_format, tile->tile_number);

    char dest_filename[FILE_MAXFILE];
    BLI_path_split_file_part(src_tile_path, dest_filename, sizeof(dest_filename));

    char dest_tile_path[FILE_MAX];
    BLI_path_join(dest_tile_path, FILE_MAX, dest_dir.c_str(), dest_filename);

    if (!allow_overwrite && BLI_exists(dest_tile_path)) {
      continue;
    }

    if (BLI_path_cmp_normalized(src_tile_path, dest_tile_path) == 0) {
      /* Source and destination paths are the same, don't copy. */
      continue;
    }

    CLOG_INFO(
        LOG_HYDRA_SCENE, 1, "Copying texture tile from %s to %s", src_tile_path, dest_tile_path);

    /* Copy the file. */
    if (BLI_copy(src_tile_path, dest_tile_path) != 0) {
      CLOG_WARN(LOG_HYDRA_SCENE,
                "USD export: could not copy texture tile from %s to %s",
                src_tile_path,
                dest_tile_path);
    }
  }
  MEM_SAFE_FREE(udim_pattern);

  return "";
}

/* Export the given texture node's image to a 'textures' directory in the export path.
 * Based on ImagesExporter::export_UV_Image() */
std::string export_texture(Image *ima, const std::string &export_path, bool allow_overwrite)
{
  char usd_dir_path[FILE_MAX];
  BLI_path_split_dir_part(export_path.c_str(), usd_dir_path, FILE_MAX);

  char tex_dir_path[FILE_MAX];
  BLI_path_join(tex_dir_path, FILE_MAX, usd_dir_path, "textures", SEP_STR);

  BLI_dir_create_recursive(tex_dir_path);

  const bool is_dirty = BKE_image_is_dirty(ima);
  const bool is_generated = ima->source == IMA_SRC_GENERATED;
  const bool is_packed = BKE_image_has_packedfile(ima);

  std::string dest_dir(tex_dir_path);
  std::string dest_path;

  if (is_generated || is_dirty || is_packed) {
    dest_path = export_in_memory_texture(ima, dest_dir, allow_overwrite);
  }
  else if (ima->source == IMA_SRC_TILED) {
    dest_path = copy_tiled_textures(ima, dest_dir, allow_overwrite);
  }
  else {
    dest_path = copy_single_file(ima, dest_dir, allow_overwrite);
  }
  return dest_path;
}

}  // namespace blender::io::hydra
