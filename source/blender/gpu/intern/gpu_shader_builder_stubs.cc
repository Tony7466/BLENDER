/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Stubs to work around a link time issue when linking on certain platforms. We should find the
 * cause and fix that so we can remove the stub file completely.
 *
 * Currently unclear why the implementations inside bf_blenkernel aren't found when compiling on
 * GCC 11.4.0. It works on clang. Didn't test newer GCCs.
 */

#include "BKE_asset.hh"
#include "DNA_asset_types.h"

AssetWeakReference::AssetWeakReference() {}
AssetWeakReference::AssetWeakReference(AssetWeakReference &&) {}
AssetWeakReference::~AssetWeakReference() {}
AssetWeakReference *AssetWeakReference::make_reference(
    const blender::asset_system::AssetLibrary & /*library*/,
    const blender::asset_system::AssetIdentifier & /*asset_identifier*/)
{
  return nullptr;
}
void BKE_asset_weak_reference_free(AssetWeakReference ** /*weak_ref*/) {}
