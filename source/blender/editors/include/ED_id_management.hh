/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BLI_compiler_attrs.h"
#include "BLI_string_ref.hh"

struct ID;
struct Main;

bool ED_id_rename(Main &bmain, ID &id, blender::StringRefNull name);
