/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"

#include "DNA_mesh_types.h"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

namespace blender::geometry {

void reorder_mesh_verts(const Span<int> indices, Mesh &mesh);
void reorder_mesh_edges(const Span<int> indices, Mesh &mesh);
void reorder_mesh_faces(const Span<int> indices, Mesh &mesh);
void reorder_points(const Span<int> indices, PointCloud &pointcloud);
void reorder_curves(const Span<int> indices, bke::CurvesGeometry &curves);
void reorder_instaces(const Span<int> indices, bke::Instances &instances);

};  // namespace blender::geometry
