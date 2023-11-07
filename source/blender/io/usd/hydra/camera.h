/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <tuple>

#include <pxr/base/gf/camera.h>
#include <pxr/base/gf/vec2f.h>

#include "BKE_camera.h"

struct ARegion;
struct Object;
struct View3D;
struct Depsgraph;
struct RenderData;

namespace blender::io::hydra {

pxr::GfCamera gf_camera(const Depsgraph *depsgraph,
                        const View3D *v3d,
                        const ARegion *region,
                        const RenderData *rd,
                        pxr::GfVec4f tile);

pxr::GfCamera gf_camera(const Object *camera_obj,
                        const RenderData *rd,
                        pxr::GfVec2i res,
                        pxr::GfVec4f tile);
}  // namespace blender::io::hydra
