/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <mutex>

#include "BKE_scene_writeback_sync.hh"

#include "DEG_depsgraph.hh"

#include "BLI_map.hh"

namespace blender::bke::scene::sync_writeback {

struct WritebacksMap {
  std::mutex mutex;
  Map<const Depsgraph *, Vector<std::function<void()>>> map;
};

static WritebacksMap &get_writebacks_map()
{
  static WritebacksMap map;
  return map;
}

void activate(const Depsgraph &depsgraph)
{
  WritebacksMap &map = get_writebacks_map();
  std::lock_guard lock{map.mutex};
  map.map.lookup_or_add_default(&depsgraph).clear();
}

void add(const Depsgraph &depsgraph, std::function<void()> fn)
{
  if (!DEG_is_active(&depsgraph)) {
    return;
  }
  WritebacksMap &map = get_writebacks_map();
  std::lock_guard lock{map.mutex};
  if (!map.map.contains(&depsgraph)) {
    return;
  }
  map.map.lookup(&depsgraph).append(std::move(fn));
}

void run(const Depsgraph &depsgraph)
{
  WritebacksMap &map = get_writebacks_map();
  Vector<std::function<void()>> writebacks;
  {
    std::lock_guard lock{map.mutex};
    if (!map.map.contains(&depsgraph)) {
      return;
    }
    writebacks = map.map.pop(&depsgraph);
  }
  for (std::function<void()> &fn : writebacks) {
    fn();
  }
}

}  // namespace blender::bke::scene::sync_writeback
