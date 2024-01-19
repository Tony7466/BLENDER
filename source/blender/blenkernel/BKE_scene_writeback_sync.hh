/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 *
 * This file provides an API that can be used to modify original (as opposed to evaluated)
 * data-blocks after depsgraph evaluation. For some data (e.g. animated properties), this is done
 * during depsgraph evaluation. However, this is not possible in all cases. For example, if the
 * change to the original data adds a new relation between data-blocks, a user-count (#ID.us) has
 * to be increased. This counter is not atomic and can therefore not be modified arbitrarily from
 * different threads.
 */

#include <functional>

struct Depsgraph;

namespace blender::bke::scene::sync_writeback {

/**
 * Enable gathering of writeback tasks before depsgraph evaluation.
 */
void activate(const Depsgraph &depsgraph);

/**
 * Add a writeback task during depsgraph evaluation. The given function is called after depsgraph
 * evaluation is done. It is allowed to change the original data blocks.
 */
void add(const Depsgraph &depsgraph, std::function<void()> fn);

/**
 * Execute all gathered writeback tasks for the given depsgraph. This should be called after
 * depsgraph evaluation finished.
 *
 * \return True if any original data was modified and the depsgraph might have to be updated.
 */
bool run(const Depsgraph &depsgraph);

}  // namespace blender::bke::scene::sync_writeback
