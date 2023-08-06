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

/**
 * This file contains utilities for converting between UsdSkel data and
 * Blender armatures and shape keys. The following is a reference on the
 * UsdSkel API:
 *
 * https://openusd.org/23.05/api/usd_skel_page_front.html
 */

/**
 * Import USD blend shapes from a USD primitive as shape keys on a mesh
 * object. If the blend shapes have animating weights, the time-sampled
 * weights will be imported as shape key animation curves. If the USD
 * primitive does not have blend shape targets defined, this function is a
 * no-op.
 *
 * \param bmain: Main pointer
 * \param obj: Mesh object to which imported shape keys will be added
 * \param prim: The USD primitive from which blendshapes will be imported
 */
void import_blendshapes(Main *bmain, Object *obj, pxr::UsdPrim prim);

void import_skeleton(Main *bmain, Object *obj, const pxr::UsdSkelSkeleton &skel);

void create_skeleton_curves(Main *bmain,
                            Object *obj,
                            const pxr::UsdSkelSkeletonQuery &skel_query,
                            const std::map<pxr::TfToken, std::string> &joint_to_bone_map);

void import_skel_bindings(Main *bmain, Object *shape_obj, pxr::UsdPrim prim);

}  // namespace blender::io::usd
