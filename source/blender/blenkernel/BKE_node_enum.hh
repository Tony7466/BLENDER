/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BLI_implicit_sharing.hh"
#include "BLI_vector.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Runtime enum items list.
 * \{ */

/**
 * Runtime copy of #NodeEnumItem for use in #RuntimeNodeEnumItems.
 */
struct RuntimeNodeEnumItem {
  std::string name;
  std::string description;
  /* Immutable unique identifier. */
  int identifier;
};

/**
 * Shared immutable list of enum items.
 * These are owned by a node and can be referenced by node sockets.
 */
struct RuntimeNodeEnumItems : blender::ImplicitSharingMixin {
  blender::Vector<RuntimeNodeEnumItem> items;

  void delete_self() override
  {
    delete this;
  }
};

/** \} */

}  // namespace blender::bke
