/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <atomic>

#include "BLI_function_ref.hh"
#include "BLI_utility_mixins.hh"

struct bContext;

namespace blender::cancellable_worker {

void run_cancellable(bContext *C, FunctionRef<void()> fn);

}  // namespace blender::cancellable_worker
