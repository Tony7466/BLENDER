/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <atomic>

#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

namespace blender::bke {

/**
 * A set of anonymous attribute names that is passed around in geometry nodes.
 */
class AnonymousAttributeSet {
 public:
  /**
   * This uses `std::shared_ptr` because attributes sets are passed around by value during geometry
   * nodes evaluation, and this makes it very small if there is no name. Also it makes copying very
   * cheap.
   */
  std::shared_ptr<Set<std::string>> names;
};

}  // namespace blender::bke
