/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct Mesh;
namespace blender {
namespace bke {
class AttributeIDRef;
}
}  // namespace blender

namespace blender::geometry {

Mesh *create_icosphere_mesh(int resolution, float radius, const bke::AttributeIDRef &uv_id);

}  // namespace blender::geometry
