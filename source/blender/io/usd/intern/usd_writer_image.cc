/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_writer_image.h"

#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BKE_image.h"
#include "BKE_image_format.h"
#include "BKE_image_save.h"
#include "BKE_main.h"
#include "BKE_packedFile.h"
#include "BKE_report.h"

#include "BLI_memory_utils.hh"

#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"

namespace blender::io::usd {

/* Get the absolute filepath of the given image.  Assumes
 * r_path result array is of length FILE_MAX. */
static void get_absolute_path(Image *ima, char *r_path)
{
  /* Make absolute source path. */
  BLI_strncpy(r_path, ima->filepath, FILE_MAX);
  BLI_path_abs(r_path, ID_BLEND_PATH_FROM_GLOBAL(&ima->id));
  BLI_path_normalize(r_path);
}

std::string get_tex_image_asset_filepath(Image *ima)
{
  char filepath[FILE_MAX];
  get_absolute_path(ima, filepath);

  return std::string(filepath);
}

std::string get_in_memory_texture_filename(Image *ima)
{
  bool is_dirty = BKE_image_is_dirty(ima);
  bool is_generated = ima->source == IMA_SRC_GENERATED;
  bool is_packed = BKE_image_has_packedfile(ima);
  if (!(is_generated || is_dirty || is_packed)) {
    return "";
  }

  /* Determine the correct file extension from the image format. */
  ImBuf *imbuf = BKE_image_acquire_ibuf(ima, nullptr, nullptr);
  if (!imbuf) {
    return "";
  }

  ImageFormatData imageFormat;
  BKE_image_format_from_imbuf(&imageFormat, imbuf);
  BKE_image_release_ibuf(ima, imbuf, nullptr);

  char file_name[FILE_MAX];
  /* Use the image name for the file name. */
  STRNCPY(file_name, ima->id.name + 2);

  BKE_image_path_ext_from_imformat_ensure(file_name, sizeof(file_name), &imageFormat);

  return file_name;
}

/* Copy the given image to the destination directory. */
static std::string copy_single_file(Image *ima,
                                    const std::string &dest_dir,
                                    const bool allow_overwrite,
                                    ReportList *reports)
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

  BKE_reportf(reports, RPT_INFO, "Copying texture from %s to %s", source_path, dest_path);

  /* Copy the file. */
  if (BLI_copy(source_path, dest_path) != 0) {
    BKE_reportf(reports,
                RPT_WARNING,
                "USD export: could not copy texture from %s to %s",
                source_path,
                dest_path);
    return "";
  }

  return dest_path;
}

static std::string export_in_memory_texture(Image *ima,
                                            const std::string &export_dir,
                                            const bool allow_overwrite,
                                            ReportList *reports)
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

  if (!allow_overwrite && BLI_exists(export_path) && !BKE_image_is_dirty(ima)) {
    return export_path;
  }

  if ((BLI_path_cmp_normalized(export_path, image_abs_path) == 0) && BLI_exists(image_abs_path)) {
    /* As a precaution, don't overwrite the original path. */
    return image_abs_path;
  }

  BKE_reportf(reports, RPT_INFO, "Exporting in-memory texture to %s", export_path);

  if (BKE_imbuf_write_as(imbuf, export_path, &imageFormat, true) == 0) {
    BKE_reportf(
        reports, RPT_WARNING, "USD export: couldn't export in-memory texture to %s", export_path);
  }

  return export_path;
}

/* If the given image is tiled, copy the image tiles to the given
 * destination directory. */
static std::string copy_tiled_textures(Image *ima,
                                       const std::string &dest_dir,
                                       const bool allow_overwrite,
                                       ReportList *reports)
{
  char src_path[FILE_MAX];
  get_absolute_path(ima, src_path);

  eUDIM_TILE_FORMAT tile_format;
  char *udim_pattern = BKE_image_get_tile_strformat(src_path, &tile_format);

  /* Only <UDIM> tile formats are supported by USD right now. */
  if (tile_format != UDIM_TILE_FORMAT_UDIM) {
    BKE_reportf(reports, RPT_WARNING, "Unsupported tile format for `%s`", src_path);
    MEM_SAFE_FREE(udim_pattern);
    return "";
  }

  std::string dest_path;

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

    BKE_reportf(
        reports, RPT_INFO, "Copying texture tile from %s to %s", src_tile_path, dest_tile_path);

    /* Copy the file. */
    if (BLI_copy(src_tile_path, dest_tile_path) != 0) {
      BKE_reportf(reports,
                  RPT_WARNING,
                  "USD export: could not copy texture tile from %s to %s",
                  src_tile_path,
                  dest_tile_path);
    }
    if (dest_path.empty()) {
      dest_path = dest_tile_path;
    }
  }
  MEM_SAFE_FREE(udim_pattern);

  return dest_path;
}

/* Export the given texture node's image to a 'textures' directory in the export path.
 * Based on ImagesExporter::export_UV_Image() */
std::string export_texture(Image *ima,
                           const std::string &export_path,
                           bool allow_overwrite,
                           bool only_in_memory,
                           ReportList *reports)
{
  const bool is_dirty = BKE_image_is_dirty(ima);
  const bool is_generated = ima->source == IMA_SRC_GENERATED;
  const bool is_packed = BKE_image_has_packedfile(ima);

  std::string dest_path;
  if (is_generated || is_dirty || is_packed) {
    BLI_dir_create_recursive(export_path.c_str());
    dest_path = export_in_memory_texture(ima, export_path, allow_overwrite, reports);
  }
  else if (only_in_memory) {
    dest_path = get_tex_image_asset_filepath(ima);
  }
  else {
    BLI_dir_create_recursive(export_path.c_str());
    if (ima->source == IMA_SRC_TILED) {
      dest_path = copy_tiled_textures(ima, export_path, allow_overwrite, reports);
    }
    else {
      dest_path = copy_single_file(ima, export_path, allow_overwrite, reports);
    }
  }
  return dest_path;
}

}  // namespace blender::io::usd
