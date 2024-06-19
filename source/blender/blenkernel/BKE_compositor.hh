/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BLI_set.hh"

struct Scene;

namespace blender::bke::compositor {

/* Get the set of all passes used by the viewport compositor, identified by their pass names. */
Set<std::string> get_used_passes(const Scene &scene);

}  // namespace blender::bke::compositor
