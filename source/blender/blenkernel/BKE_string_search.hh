/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_search.hh"

namespace blender::bke::string_search {

void add_recent_search(StringRef choosen_str);

const blender::string_search::RecentCache *get_recent_cache();

template<typename T> class StringSearch : public blender::string_search::StringSearch<T> {
 public:
  StringSearch() : blender::string_search::StringSearch<T>(get_recent_cache()) {}
};

}  // namespace blender::bke::string_search
