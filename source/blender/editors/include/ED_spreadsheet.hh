/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct ID;
struct SpaceSpreadsheet;

#define TOP_ROW_HEIGHT (UI_UNIT_Y * 1.1f)

namespace blender::ed::spreadsheet {

ID *get_current_id(const SpaceSpreadsheet *sspreadsheet);
rcti get_layout_maskrect(const SpaceSpreadsheet &sspreadsheet, const ARegion &region);

}  // namespace blender::ed::spreadsheet
