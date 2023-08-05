/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"

namespace blender::bke::string_search {

void add_recent_search(StringRef search_id, StringRef choosen_str);

}
