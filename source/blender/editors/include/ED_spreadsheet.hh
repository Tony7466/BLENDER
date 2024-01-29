/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct ID;
struct SpaceSpreadsheet;

#define TOP_ROW_HEIGHT (UI_UNIT_Y * 1.1f)

ID *ED_spreadsheet_get_current_id(const SpaceSpreadsheet *sspreadsheet);
void ED_spreadsheet_layout_maskrect(const SpaceSpreadsheet *sspreadsheet,
                                    const ARegion *region,
                                    rcti *r_rect);
