/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#if defined(WITH_COLLADA) || defined(WITH_IO_GPENCIL) || defined(WITH_IO_WAVEFRONT_OBJ) || \
    defined(WITH_IO_PLY) || defined(WITH_IO_STL) || defined(WITH_USD)

#  include "WM_types.h"

#  ifdef __cplusplus
extern "C" {
#  endif

struct wmOperator;
struct wmOperatorType;
struct wmDrag;
struct wmDropBox;

int wm_io_import_invoke(bContext *C, wmOperator *op, const wmEvent *event);

void skip_save_import_paths_props(wmOperatorType *ot, const eFileSel_Flag flag);

void files_drop_copy(bContext *C, wmDrag *drag, wmDropBox *drop);

void files_drop_info_draw(bContext *C, wmOperator *op, int icon);

#  ifdef __cplusplus
}
#  endif

#endif
