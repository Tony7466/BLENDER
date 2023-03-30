/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_modifier_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void BKE_modifier_simulation_cache_invalidate(ModifierSimulationCacheHandle *cache);

#ifdef __cplusplus
}
#endif
