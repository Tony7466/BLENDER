/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#ifdef WITH_JOLT
#  include <Jolt/Jolt.h>

#  include <Jolt/Core/JobSystem.h>
#  include <Jolt/Core/JobSystemWithBarrier.h>
#endif

namespace blender::bke {

void jolt_physics_init();
void jolt_physics_exit();

#ifdef WITH_JOLT

/* TODO Implement a custom JobSystem interface for Jolt. */

#endif

}  // namespace blender::bke
