/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <optional>

#include "BLI_string_ref.hh"
#include "BLI_struct_equality_utils.hh"

#include "BKE_bake_data_block_id.hh"

#include "DNA_ID_enums.h"

namespace blender::bke::bake {

struct BakeDataBlockMap {
 public:
  /**
   * Tries to retrieve the data block for the given key. If it's not explicitly mapped, it might be
   * added to the mapping. If it's still not found, null is returned.
   */
  virtual ID *lookup_or_try_add(const BakeDataBlockID &key,
                                std::optional<ID_Type> type = std::nullopt) = 0;

  /**
   * Tries to add the data block to the map. This may not succeed in all cases, e.g. if the
   * implementation does not allow inserting new mapping items.
   */
  virtual void try_add(const ID &id) = 0;
};

}  // namespace blender::bke::bake
