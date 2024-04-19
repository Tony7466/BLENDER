/*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
* The Original Code is Copyright (C) 2008 Blender Foundation.
* All rights reserved.
*/

/** \file
* \ingroup edobj
*/

#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_fileops.h"
#include "BLI_listbase.h"
#include "BLI_math_base.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "DNA_object_types.h"
#include "DNA_usd_stage_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "BKE_context.hh"
//!TODO(kiki): this might need to come back when we ingest the idtype refactor
//#include "BKE_idtype.h"
//#include "BKE_lib_id.h"
//#include "BKE_lib_query.h"
//#include "BKE_lib_remap.h"
#include "BKE_library.hh"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_lib_remap.hh"

#include "BKE_usd_stage.hh"

#include "WM_types.hh"
#include "WM_api.hh"

#include "ED_image.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "object_intern.h"

/* usd_stage Add */

static Object *object_usd_stage_add(bContext *C, wmOperator *op, const char *name)
{
 float loc[3], rot[3];
 bool enter_editmode = false;
 ushort local_view_bits = 0;

 if (!ED_object_add_generic_get_opts(
     C, op, 'Z', loc, rot, nullptr, &enter_editmode, &local_view_bits, nullptr)) {
   return NULL;
 }

 Object* ob = ED_object_add_type(C, OB_USD_STAGE, name, loc, rot, false, local_view_bits);
 //!TODO(kiki): remove once there is actual drawing
 ob->dtx |= OB_DRAWBOUNDOX;
 return ob;
}

static int object_usd_stage_add_exec(bContext *C, wmOperator *op)
{
 return (object_usd_stage_add(C, op, NULL) != NULL) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void OBJECT_OT_usd_stage_add(wmOperatorType *ot)
{
 /* identifiers */
 ot->name = "Add USD Stage";
 ot->description = "Add a USD stage object to the scene";
 ot->idname = "OBJECT_OT_usd_stage_add";

 /* api callbacks */
 ot->exec = object_usd_stage_add_exec;
 ot->poll = ED_operator_objectmode;

 /* flags */
 ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

 ED_object_add_generic_props(ot, false);
}

/* USDStage Import */

static int usd_stage_import_exec(bContext *C, wmOperator *op)
{
 //!TODO(kiki): Fill this out and lean on Ben's import code
 //  Main *bmain = CTX_data_main(C);
 //  const bool is_relative_path = RNA_boolean_get(op->ptr, "relative_path");
 //  bool imported = false;
 //
 //  ListBase ranges = ED_image_filesel_detect_sequences(bmain, op, false);
 //  for (ImageFrameRange *range = ranges.first; range; range = range->next) {
 //    char filename[FILE_MAX];
 //    BLI_split_file_part(range->filepath, filename, sizeof(filename));
 //    BLI_path_extension_replace(filename, sizeof(filename), "");
 //
 //    Object *object = object_usd_stage_add(C, op, filename);
 //    USDStage* usd_stage = (USDStage*)object->data;
 //
 //    STRNCPY(usd_stage->filepath, range->filepath);
 //    if (is_relative_path) {
 //      BLI_path_rel(usd_stage->filepath, BKE_main_blendfile_path(bmain));
 //    }
 //
 //    //!TODO(kiki): support usd sequences?
 //    usd_stage->is_sequence = (range->length > 1);
 //    usd_stage->frame_duration = (usd_stage->is_sequence) ? range->length : 0;
 //    usd_stage->frame_start = 1;
 //    usd_stage->frame_offset = (usd_stage->is_sequence) ? range->offset - 1 : 0;
 //
 //    if (!BKE_usd_stage_load(usd_stage, bmain)) {
 //      BKE_reportf(op->reports,
 //                  RPT_WARNING,
 //                  "USD Stage \"%s\" failed to load: %s",
 //                  filename,
 //                  BKE_usd_stage_error_msg(usd_stage));
 //      BKE_id_delete(bmain, &object->id);
 //      BKE_id_delete(bmain, &usd_stage->id);
 //      continue;
 //    }
 //    //!TODO(kiki): error checks for unsupported data types?
 //    // else if (BKE_usd_stage_is_points_only(usd_stage)) {
 //    //   BKE_reportf(op->reports,
 //    //               RPT_WARNING,
 //    //               "usd_stage \"%s\" contains points, only voxel grids are supported",
 //    //               filename);
 //    //   BKE_id_delete(bmain, &object->id);
 //    //   BKE_id_delete(bmain, &usd_stage->id);
 //    //   continue;
 //    // }
 //
 //    if (BKE_usd_stage_is_y_up(usd_stage)) {
 //      object->rot[0] += M_PI_2;
 //    }
 //
 //    imported = true;
 //  }
 //  BLI_freelistN(&ranges);
 //
 //  return (imported) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
 return OPERATOR_CANCELLED;
}

static int usd_stage_import_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  (void)event;
 if (RNA_struct_property_is_set(op->ptr, "filepath")) {
   return usd_stage_import_exec(C, op);
 }

 RNA_string_set(op->ptr, "filepath", U.textudir);
 WM_event_add_fileselect(C, op);

 return OPERATOR_RUNNING_MODAL;
}

/* called by other space types too */
void OBJECT_OT_usd_stage_import(wmOperatorType *ot)
{
 /* identifiers */
 ot->name = "Import USD Stage";
 ot->description = "Import USD stage file";
 ot->idname = "OBJECT_OT_usd_stage_import";

 /* api callbacks */
 ot->exec = usd_stage_import_exec;
 ot->invoke = usd_stage_import_invoke;

 /* flags */
 ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

 /* properties */
 WM_operator_properties_filesel(ot,
                                FILE_TYPE_FOLDER | FILE_TYPE_USD,
                                FILE_SPECIAL,
                                FILE_OPENFILE,
                                WM_FILESEL_FILEPATH | WM_FILESEL_DIRECTORY |
                                    WM_FILESEL_FILES | WM_FILESEL_RELPATH,
                                FILE_DEFAULTDISPLAY,
                                FILE_SORT_ALPHA);

 RNA_def_boolean(
     ot->srna,
     "use_sequence_detection",
     true,
     "Detect Sequences",
     "Automatically detect animated sequences in selected USD files (based on file names)");

 ED_object_add_generic_props(ot, false);
}
