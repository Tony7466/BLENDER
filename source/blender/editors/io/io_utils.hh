/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "WM_types.hh"

struct wmOperator;
struct wmOperatorType;
struct wmDrag;
struct wmDropBox;

int io_util_import_invoke(bContext *C, wmOperator *op, const wmEvent *event);
void io_util_skip_save_filesel_props(wmOperatorType *ot, const eFileSel_Flag flag);
void io_util_drop_file_label_draw(bContext *C, wmOperator *op, int icon, const char *extension);
