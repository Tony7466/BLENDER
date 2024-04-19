/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
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
* Contributor(s): Jörg Müller.
*
* ***** END GPL LICENSE BLOCK *****
*/

/** \file blender/makesrna/intern/rna_hair.c
*  \ingroup RNA
*/

#include <stdlib.h>

#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "rna_internal.h"

#include "DNA_scene_types.h"
#include "DNA_usd_stage_types.h"

#include "BLI_math_base.h"
#include "BLI_string.h"

#ifdef RNA_RUNTIME

#  include "BLI_math_vector.h"

#  include "BKE_usd_stage.hh"

#  include "DEG_depsgraph.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

static USDStage *rna_usd_stage(PointerRNA *ptr)
{
 return (USDStage*)ptr->owner_id;
}

static void rna_usd_stage_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
  (void)bmain;
  (void)scene;
 USDStage* stage = (USDStage*)ptr->data;

 DEG_id_tag_update(&stage->id, ID_RECALC_COPY_ON_WRITE);
 WM_main_add_notifier(NC_OBJECT | ND_DRAW, NULL);
}

static void rna_usd_stage_filepath_update_cb(Main *bmain,
                                            Scene *scene,
                                            PointerRNA *ptr)
{
  (void)scene;
 USDStage* stage = (USDStage*)ptr->data;
 /**!TODO(kiki): I'm really not sure how this is supposed to work.
  * The updates are not firing when I think they should, and
  * frame-to-frame updates linked to time don't seem to be
  * happening either, so I'm punting and calling the same
  * update method to force a recache of the paths when this
  * filename changes.
  **/
 printf("Rebuilding stage %s...\n", stage->id.name);
 stage->flags |= USD_ST_NEEDS_REBUILD;
 BKE_usd_stage_eval(bmain, stage);
 DEG_id_tag_update(&stage->id, ID_RECALC_ALL);
 WM_main_add_notifier(NC_SCENE | ND_OB_ACTIVE, NULL);
}


static void rna_usd_stage_object_paths_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
 USDStage* stage = (USDStage*)ptr->data;
 rna_iterator_listbase_begin(iter, &stage->object_paths, NULL);
}

#else


const EnumPropertyItem rna_enum_usd_stage_prim_types_items[] = {
   {USD_ROOT_PRIM,  "ROOT_PRIM", 0, "root prim", ""},
   {USD_PRIM_XFORM, "XFORM",     0, "xform",     ""},
   {USD_PRIM_MESH,  "MESH",      0, "mesh",      ""},
   {USD_PRIM_CURVE, "CURVE",     0, "curve",     ""},
   {USD_PRIM_JOINT, "JOINT",     0, "joint",     ""},
   {0, NULL, 0, NULL, NULL},
};

const EnumPropertyItem rna_enum_usd_stage_purpose_items[] = {
   {USD_PURPOSE_NONE,   "NONE",    0, "none",    ""},
   {USD_PURPOSE_GUIDE,  "GUIDE",   0, "guide",   ""},
   {USD_PURPOSE_PROXY,  "PROXY",   0, "proxy",   ""},
   {USD_PURPOSE_RENDER, "RENDER",  0, "render",  ""},
   {0, NULL, 0, NULL, NULL},
};

const EnumPropertyItem rna_enum_usd_stage_error_items[] = {
   {USD_ERR_NO_ERROR, "ERR_NONE",      0, "No Error", ""},
   {USD_ERR_NO_FILE,  "ERR_NO_FILE",   0, "File not found", ""},
   {USD_ERR_NO_FILE,  "ERR_CANT_OPEN", 0, "Can't open file", ""},
};

static void rna_def_usd_stage_object_path(BlenderRNA *brna)
{
 StructRNA *srna = RNA_def_struct(brna, "USDStagePrimPath", NULL);
 RNA_def_struct_sdna(srna, "USDStagePrimPath");
 RNA_def_struct_ui_text(srna, "Prim Path", "Path of a prim inside of a USD Stage");
 RNA_def_struct_ui_icon(srna, ICON_NONE);

 PropertyRNA *prop = RNA_def_property(srna, "path", PROP_STRING, PROP_NONE);
 RNA_def_property_ui_text(prop, "Path", "Object path");
 RNA_def_struct_name_property(srna, prop);

 prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "type");
 RNA_def_property_enum_items(prop, rna_enum_usd_stage_prim_types_items);
 RNA_def_property_ui_text(prop, "Type", "");

 prop = RNA_def_property(srna, "parent_index", PROP_INT, PROP_NONE);
 RNA_def_property_int_sdna(prop, NULL, "parent_index");
 RNA_def_property_ui_text(prop, "parent_index", "");
 RNA_def_property_int_default(prop, -1);

 prop = RNA_def_property(srna, "active", PROP_BOOLEAN, PROP_NONE);
 RNA_def_property_boolean_sdna(prop, NULL, "flags", USD_PATH_ACTIVE);
 RNA_def_property_ui_text(prop, "active", "Is the prim currently active");
 //!TODO(kiki): should this toggle something internally in the stage?
 // RNA_def_property_update(prop, 0, "rna_userdef_script_autoexec_update");

 prop = RNA_def_property(srna, "is_loaded", PROP_BOOLEAN, PROP_NONE);
 RNA_def_property_boolean_sdna(prop, NULL, "flags", USD_PATH_LOADED);
 RNA_def_property_ui_text(prop, "is_loaded", "Is the prim currently loaded");
 //!TODO(kiki): should this toggle something internally in the stage?
 // RNA_def_property_update(prop, 0, "rna_userdef_script_autoexec_update");

 prop = RNA_def_property(srna, "is_visible", PROP_BOOLEAN, PROP_NONE);
 RNA_def_property_boolean_sdna(prop, NULL, "flags", USD_PATH_VISIBLE);
 RNA_def_property_ui_text(prop, "is_visible", "Is the prim currently visible");
 //!TODO(kiki): should this toggle something internally in the stage?
 // RNA_def_property_update(prop, 0, "rna_userdef_script_autoexec_update");
}

static void rna_def_usd_stage_object_paths(BlenderRNA *brna, PropertyRNA *cprop)
{
 RNA_def_property_srna(cprop, "USDStagePrimPaths");
 StructRNA *srna = RNA_def_struct(brna, "USDStagePrimPaths", NULL);
 RNA_def_struct_sdna(srna, "USDStage");
 RNA_def_struct_ui_text(srna, "Prim Paths", "Collection of prim paths");
}

static void rna_def_usd_stage_object(BlenderRNA *brna)
{
 StructRNA *srna;
 PropertyRNA *prop;

 srna = RNA_def_struct(brna, "USDStage", "ID");
 RNA_def_struct_ui_text(srna, "USDStage", "USDStage data-block for interop with USD files");
 //!TODO(kiki): Real icon!
 RNA_def_struct_ui_icon(srna, ICON_X);

 // not using prop_string here so we can customize the file chooser in the UI
 prop = RNA_def_property(srna, "filepath", PROP_STRING, PROP_NONE);
 RNA_def_property_string_sdna(prop, NULL, "filepath");
 RNA_def_property_ui_text(prop, "File Path", "Path to external displacements file");
 RNA_def_property_update(prop, 0, "rna_usd_stage_filepath_update_cb");

 prop = RNA_def_property(srna, "is_sequence", PROP_BOOLEAN, PROP_NONE);
 RNA_def_property_ui_text(
     prop, "Sequence", "Whether the stage is separated in a series of files");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 /* ----------------- For Scene time ------------------- */

 prop = RNA_def_property(srna, "override_frame", PROP_BOOLEAN, PROP_NONE);
 RNA_def_property_ui_text(prop,
                          "Override Frame",
                          "Whether to use a custom frame for looking up data in the USD file,"
                          " instead of using the current scene frame");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "error", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "error");
 RNA_def_property_enum_items(prop, rna_enum_usd_stage_error_items);
 RNA_def_property_ui_text(prop, "Error", "");
 RNA_def_property_clear_flag(prop, PROP_EDITABLE);

 prop = RNA_def_property(srna, "frame", PROP_FLOAT, PROP_NONE);
 RNA_def_property_float_sdna(prop, NULL, "frame");
 RNA_def_property_range(prop, -MAXFRAME, MAXFRAME);
 RNA_def_property_ui_text(prop,
                          "Frame",
                          "The time to use for looking up the data in the cache file,"
                          " or to determine which file to use in a file sequence");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "frame_offset", PROP_FLOAT, PROP_NONE);
 RNA_def_property_float_sdna(prop, NULL, "frame_offset");
 RNA_def_property_range(prop, -MAXFRAME, MAXFRAME);
 RNA_def_property_ui_text(prop,
                          "Frame Offset",
                          "Subtracted from the current frame to use for "
                          "looking up the data in the cache file, or to "
                          "determine which file to use in a file sequence");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 /* ----------------- Axis Conversion ----------------- */

 prop = RNA_def_property(srna, "forward_axis", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "forward_axis");
 RNA_def_property_enum_items(prop, rna_enum_object_axis_items);
 RNA_def_property_ui_text(prop, "Forward", "");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "up_axis", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "up_axis");
 RNA_def_property_enum_items(prop, rna_enum_object_axis_items);
 RNA_def_property_ui_text(prop, "Up", "");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "scale", PROP_FLOAT, PROP_NONE);
 RNA_def_property_float_sdna(prop, NULL, "scale");
 RNA_def_property_range(prop, 0.0001f, 1000.0f);
 RNA_def_property_ui_text(
     prop,
     "Scale",
     "Value by which to enlarge or shrink the object with respect to the world's origin"
     " (only applicable through a Transform Cache constraint)");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "root_prim_path", PROP_STRING, PROP_NONE);
 RNA_def_property_string_sdna(prop, NULL, "root_prim_path");
 RNA_def_property_ui_text(prop, "Path to root prim in the USD Stage to display", "");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 /* materials */
 prop = RNA_def_property(srna, "materials", PROP_COLLECTION, PROP_NONE);
 RNA_def_property_collection_sdna(prop, NULL, "mat", "totcol");
 RNA_def_property_struct_type(prop, "Material");
 RNA_def_property_ui_text(prop, "Materials", "");
 RNA_def_property_srna(prop, "IDMaterials"); /* see rna_ID.c */
 RNA_def_property_collection_funcs(
     prop, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "rna_IDMaterials_assign_int");

 /** prim paths **/
 prop = RNA_def_property(srna, "object_paths", PROP_COLLECTION, PROP_NONE);
 RNA_def_property_collection_sdna(prop, NULL, "object_paths", NULL);
 RNA_def_property_collection_funcs(prop,
                                   "rna_usd_stage_object_paths_begin",
                                   "rna_iterator_listbase_next",
                                   "rna_iterator_listbase_end",
                                   "rna_iterator_listbase_get",
                                   NULL,
                                   NULL,
                                   NULL,
                                   NULL);
 RNA_def_property_struct_type(prop, "USDStagePrimPath");
 RNA_def_property_srna(prop, "USDStagePrimPaths");
 RNA_def_property_ui_text(
     prop, "Prim Paths", "Paths of the prims inside the USD Stage");
 rna_def_usd_stage_object_paths(brna, prop);

 /** active prim properties **/
 prop = RNA_def_property(srna, "active_prim_path", PROP_STRING, PROP_NONE);
 RNA_def_property_string_sdna(prop, NULL, "active_prim_path");
 RNA_def_property_ui_text(prop, "Active Prim Path", "Path to active prim for the interface");
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "active_prim_variant", PROP_STRING, PROP_NONE);
 RNA_def_property_string_sdna(prop, NULL, "active_prim_variant");
 RNA_def_property_ui_text(prop, "Active Prim Variant", "Chosen variant for active prim");
 //!TODO(kiki): Make set change the session edit layer
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 prop = RNA_def_property(srna, "active_prim_matrix_local", PROP_FLOAT, PROP_MATRIX);
 RNA_def_property_float_sdna(prop, NULL, "active_prim_matrix_local");
 RNA_def_property_multi_array(prop, 2, rna_matrix_dimsize_4x4);
 RNA_def_property_ui_text(prop, "Active Prim Matrix Local", "Local matrix of active prim");
 RNA_def_property_clear_flag(prop, PROP_EDITABLE);

 prop = RNA_def_property(srna, "active_prim_matrix_world", PROP_FLOAT, PROP_MATRIX);
 RNA_def_property_float_sdna(prop, NULL, "active_prim_matrix_world");
 RNA_def_property_multi_array(prop, 2, rna_matrix_dimsize_4x4);
 RNA_def_property_ui_text(prop, "Active Prim World Matrix", "World matrix of active prim");
 RNA_def_property_clear_flag(prop, PROP_EDITABLE);

 prop = RNA_def_property(srna, "active_prim_type", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "active_prim_type");
 RNA_def_property_enum_items(prop, rna_enum_usd_stage_prim_types_items);
 RNA_def_property_ui_text(prop, "Active Prim Type", "USD type for active prim");
 RNA_def_property_clear_flag(prop, PROP_EDITABLE);

 prop = RNA_def_property(srna, "active_prim_purpose", PROP_ENUM, PROP_NONE);
 RNA_def_property_enum_sdna(prop, NULL, "active_prim_purpose");
 RNA_def_property_enum_items(prop, rna_enum_usd_stage_purpose_items);
 RNA_def_property_ui_text(prop, "Active Prim Purpose", "USD purpose for active prim");
 //!TODO(kiki): Make set change the session edit layer
 RNA_def_property_update(prop, 0, "rna_usd_stage_update");

 /* common */
 rna_def_animdata_common(srna);
}

void RNA_def_usd_stage(BlenderRNA *brna)
{
 rna_def_usd_stage_object(brna);
 rna_def_usd_stage_object_path(brna);
}

#endif
