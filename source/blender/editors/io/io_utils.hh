/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "WM_types.hh"

struct wmOperator;
struct wmOperatorType;
struct wmDrag;
struct wmDropBox;

namespace blender::bke {
struct FileHanlderType;
}  // namespace blender::bke

namespace blender::ed::io {
/**
 * Shows a import dialog if the operator was invoked with filepath properties set, otherwise
 * invokes the fileselect window.
 */
int filesel_drop_import_invoke(bContext *C, wmOperator *op, const wmEvent *event);

/**
 * Shows filepath information if the operator is running as a dialog popup.
 */
void filepath_label_draw(bContext *C, wmOperator *op);

bool poll_file_object_drop(const bContext *C, blender::bke::FileHandlerType *fh);
}  // namespace blender::ed::io
