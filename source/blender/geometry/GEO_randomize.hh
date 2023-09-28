/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct Mesh;

namespace blender::geometry {

void randomize_vertex_order(Mesh &mesh);
void randomize_edge_order(Mesh &mesh);
void randomize_face_order(Mesh &mesh);

};  // namespace blender::geometry
