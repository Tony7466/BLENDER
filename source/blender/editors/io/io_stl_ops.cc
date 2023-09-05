/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editor/io
 */

#ifdef WITH_IO_STL

#  include "BLT_translation.h"

#  include "BKE_context.h"
#  include "BKE_report.h"

#  include "WM_api.hh"
#  include "WM_types.hh"

#  include "DNA_space_types.h"

#  include "ED_outliner.hh"

#  include "RNA_access.hh"
#  include "RNA_define.hh"

#  include "UI_interface.hh"
#  include "UI_resources.hh"

#  include "UI_interface.hh"
#  include "UI_resources.hh"

#  include "UI_interface.hh"
#  include "UI_resources.hh"

#  include "IO_stl.hh"
#  include "io_stl_ops.hh"
#  include "io_utils.hh"

static int wm_stl_import_exec(bContext *C, wmOperator *op)
{
  STLImportParams params{};
  params.forward_axis = eIOAxis(RNA_enum_get(op->ptr, "forward_axis"));
  params.up_axis = eIOAxis(RNA_enum_get(op->ptr, "up_axis"));
  params.use_facet_normal = RNA_boolean_get(op->ptr, "use_facet_normal");
  params.use_scene_unit = RNA_boolean_get(op->ptr, "use_scene_unit");
  params.global_scale = RNA_float_get(op->ptr, "global_scale");
  params.use_mesh_validate = RNA_boolean_get(op->ptr, "use_mesh_validate");

  int files_len = RNA_collection_length(op->ptr, "files");

  if (files_len) {
    PointerRNA fileptr;
    PropertyRNA *prop;
    char dir_only[FILE_MAX], file_only[FILE_MAX];

    RNA_string_get(op->ptr, "directory", dir_only);
    prop = RNA_struct_find_property(op->ptr, "files");
    for (int i = 0; i < files_len; i++) {
      RNA_property_collection_lookup_int(op->ptr, prop, i, &fileptr);
      RNA_string_get(&fileptr, "name", file_only);
      BLI_path_join(params.filepath, sizeof(params.filepath), dir_only, file_only);
      STL_import(C, &params);
    }
  }
  else if (RNA_struct_property_is_set_ex(op->ptr, "filepath", false)) {
    RNA_string_get(op->ptr, "filepath", params.filepath);
    STL_import(C, &params);
  }
  else {
    BKE_report(op->reports, RPT_ERROR, "No filepath given");
    return OPERATOR_CANCELLED;
  }

  Scene *scene = CTX_data_scene(C);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, scene);
  ED_outliner_select_sync_from_object_tag(C);

  return OPERATOR_FINISHED;
}

static bool wm_stl_import_check(bContext * /*C*/, wmOperator *op)
{
  const int num_axes = 3;
  /* Both forward and up axes cannot be the same (or same except opposite sign). */
  if (RNA_enum_get(op->ptr, "forward_axis") % num_axes ==
      (RNA_enum_get(op->ptr, "up_axis") % num_axes))
  {
    RNA_enum_set(op->ptr, "up_axis", RNA_enum_get(op->ptr, "up_axis") % num_axes + 1);
    return true;
  }
  return false;
}
static void ui_stl_import_settings(uiLayout *layout, PointerRNA *imfptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);

  uiLayout *box = uiLayoutBox(layout);
  uiLayout *col = uiLayoutColumn(box, false);
  uiItemR(col, imfptr, "global_scale", UI_ITEM_NONE, NULL, ICON_NONE);
  uiItemR(col, imfptr, "use_scene_unit", UI_ITEM_NONE, NULL, ICON_NONE);
  uiItemR(col, imfptr, "use_facet_normal", UI_ITEM_NONE, NULL, ICON_NONE);
  uiItemR(col, imfptr, "forward_axis", UI_ITEM_NONE, IFACE_("Forward Axis"), ICON_NONE);
  uiItemR(col, imfptr, "up_axis", UI_ITEM_NONE, NULL, ICON_NONE);
  uiItemR(col, imfptr, "use_mesh_validate", UI_ITEM_NONE, NULL, ICON_NONE);
}

static void wm_stl_import_draw(bContext *C, wmOperator *op)
{
  PointerRNA ptr;
  wmWindowManager *wm = CTX_wm_manager(C);
  RNA_pointer_create(&wm->id, op->type->srna, op->properties, &ptr);
  files_drop_label_draw(C, op, ICON_FILE_3D, ".stl");
  ui_stl_import_settings(op->layout, &ptr);
}

void WM_OT_stl_import(wmOperatorType *ot)
{
  PropertyRNA *prop;

  ot->name = "Import STL";
  ot->description = "Import an STL file as an object";
  ot->idname = "WM_OT_stl_import";

  ot->invoke = wm_io_import_invoke;
  ot->exec = wm_stl_import_exec;
  ot->poll = WM_operator_winactive;
  ot->check = wm_stl_import_check;
  ot->ui = wm_stl_import_draw;
  ot->flag = OPTYPE_UNDO | OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  skip_filesel_props(ot, WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY);

  RNA_def_float(ot->srna, "global_scale", 1.0f, 1e-6f, 1e6f, "Scale", "", 0.001f, 1000.0f);
  RNA_def_boolean(ot->srna,
                  "use_scene_unit",
                  false,
                  "Scene Unit",
                  "Apply current scene's unit (as defined by unit scale) to imported data");
  RNA_def_boolean(ot->srna,
                  "use_facet_normal",
                  false,
                  "Facet Normals",
                  "Use (import) facet normals (note that this will still give flat shading)");
  RNA_def_enum(ot->srna, "forward_axis", io_transform_axis, IO_AXIS_Y, "Forward Axis", "");
  RNA_def_enum(ot->srna, "up_axis", io_transform_axis, IO_AXIS_Z, "Up Axis", "");
  RNA_def_boolean(ot->srna,
                  "use_mesh_validate",
                  false,
                  "Validate Mesh",
                  "Validate and correct imported mesh (slow)");

  /* Only show .stl files by default. */
  prop = RNA_def_string(ot->srna, "filter_glob", "*.stl", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

#endif /* WITH_IO_STL */
