/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_gizmos.hh"

namespace blender::bke {

GizmosGeometry::GizmosGeometry(std::string path) : paths_{std::move(path)}, mapping(1, 0)
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  attributes_.reallocate(1);
}

GizmosGeometry *GizmosGeometry::copy() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  GizmosGeometry *gizmos = new GizmosGeometry;
  gizmos->paths_ = paths_;
  gizmos->mapping = mapping;
  gizmos->attributes_ = attributes_;
  return gizmos;
}

int GizmosGeometry::pathes_num() const
{
  return paths_.size();
}

int GizmosGeometry::gizmos_num() const
{
  return mapping.size();
}

}  // namespace blender::bke
