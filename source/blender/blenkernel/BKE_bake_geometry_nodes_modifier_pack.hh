/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_modifier_types.h"

#include "BKE_bake_items_paths.hh"

struct ReportList;

namespace blender::bke::bake {

NodesModifierPackedBake *pack_bake_from_disk(const BakePath &bake_path, ReportList *reports);

[[nodiscard]] bool unpack_bake_to_disk(const NodesModifierPackedBake &packed_bake,
                                       const BakePath &bake_path,
                                       ReportList *reports);

}  // namespace blender::bke::bake
