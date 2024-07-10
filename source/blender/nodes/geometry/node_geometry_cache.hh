/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

namespace blender::bke {
struct GeometrySet;
}

namespace blender {
void geo_cache_set(std::string key, bke::GeometrySet geometry);
bool geo_cache_contains(std::string key);
void geo_cache_clear(std::string key);
void geo_cache_clear_all();
bke::GeometrySet geo_cache_get(std::string key);
}  // namespace blender
