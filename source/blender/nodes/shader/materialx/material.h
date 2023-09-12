/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

struct Depsgraph;
struct Material;

namespace blender::nodes::materialx {

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material);

}  // namespace blender::nodes::materialx
