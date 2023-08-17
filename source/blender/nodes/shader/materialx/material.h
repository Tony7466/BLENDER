/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

#include "DNA_material_types.h"

namespace blender::nodes::materialx {

MaterialX::DocumentPtr export_to_materialx(Material *material);

}  // namespace blender::materialx

