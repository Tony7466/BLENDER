/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <optional>

#include "BLI_string_ref.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_ID_enums.h"

struct ID;

namespace blender::bke::bake {

struct BakeDataBlockKey {
  StringRef id_name;
  /**
   * This is a name of the Library ID data block, not the file path. It is
   * necessary to unique identify an ID in the .blend file.
   */
  StringRef lib_name;

  BakeDataBlockKey() = default;
  BakeDataBlockKey(StringRef id_name, StringRef lib_name);
  BakeDataBlockKey(const ID &id);

  uint64_t hash() const;

  BLI_STRUCT_EQUALITY_OPERATORS_2(BakeDataBlockKey, id_name, lib_name)
};

struct BakeDataBlockMap {
 public:
  /**
   * Tries to retrieve the data block for the given key. If it's not explicitly mapped, it might be
   * added to the mapping. If it's still not found, null is returned.
   */
  virtual ID *lookup_or_try_add(const BakeDataBlockKey &key,
                                std::optional<ID_Type> type = std::nullopt) = 0;

  /**
   * Tries to add the data block to the map. This may not succeed in all cases, e.g. if the
   * implementation does not allow inserting new mapping items.
   */
  virtual void try_add(const ID &id) = 0;
};

}  // namespace blender::bke::bake
