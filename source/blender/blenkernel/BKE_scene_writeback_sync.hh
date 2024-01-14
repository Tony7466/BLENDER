/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <functional>

struct Depsgraph;

namespace blender::bke::scene::sync_writeback {

void activate(const Depsgraph &depsgraph);
void run(const Depsgraph &depsgraph);
void add(const Depsgraph &depsgraph, std::function<void()> fn);

}  // namespace blender::bke::scene::sync_writeback
