/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <string>

struct Image;
struct ReportList;

namespace blender::io::usd {

std::string get_tex_image_asset_filepath(Image *ima);

/* Generate a file name for an in-memory image that doesn't have a
 * filepath already defined. */
std::string get_in_memory_texture_filename(Image *ima);

/* Export the given texture */
std::string export_texture(Image *ima,
                           const std::string &export_path,
                           bool allow_overwrite,
                           bool only_in_memory,
                           ReportList *reports);

}  // namespace blender::io::usd
