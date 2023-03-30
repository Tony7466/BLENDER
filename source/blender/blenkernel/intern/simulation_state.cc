/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_simulation_state.h"
#include "BKE_simulation_state.hh"

void BKE_modifier_simulation_cache_invalidate(ModifierSimulationCacheHandle *cache)
{
  cache->invalidate();
}
