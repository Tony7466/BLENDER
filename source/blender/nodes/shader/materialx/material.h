/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

struct Depsgraph;
struct Material;

class ExportImageFunction;

namespace blender::nodes::materialx {

using ExportImageFunction = std::function<std::string(Main *, Scene *, Image *, ImageUser *)>;

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph,
                                           Material *material,
                                           const char *material_name,
                                           ExportImageFunction export_image_fn);

}  // namespace blender::nodes::materialx
