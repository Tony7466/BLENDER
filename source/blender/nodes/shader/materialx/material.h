/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

#include "CLG_log.h"

struct Depsgraph;
struct Material;

namespace blender::nodes::materialx {

extern struct CLG_LogRef *LOG_MATERIALX_SHADER;

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material);

}  // namespace blender::nodes::materialx
