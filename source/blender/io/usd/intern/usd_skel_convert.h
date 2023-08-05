/* SPDX-FileCopyrightText: 2023 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdSkel/skeletonQuery.h>
#include <map>

struct Main;
struct Object;
struct Scene;
struct USDExportParams;
struct USDImportParams;

namespace blender::io::usd {

struct ImportSettings;

void import_blendshapes(Main *bmain, Object *shape_obj, pxr::UsdPrim prim);

void import_skeleton(Main *bmain, Object *obj, const pxr::UsdSkelSkeleton &skel);

void create_skeleton_curves(Main *bmain,
                            Object *obj,
                            const pxr::UsdSkelSkeletonQuery &skel_query,
                            const std::map<pxr::TfToken, std::string> &joint_to_bone_map);

void import_skel_bindings(Main *bmain, Object *shape_obj, pxr::UsdPrim prim);

}  // namespace blender::io::usd
