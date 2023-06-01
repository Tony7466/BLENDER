/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

namespace blender::bke {

int GizmosGeometry::pathes_num() const
{
  return paths_.size();
}

int GizmosGeometry::gizmos_num() const
{
  return mapping.size();
}

}  // namespace blender::bke
