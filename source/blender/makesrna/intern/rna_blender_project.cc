/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "RNA_define.hh"

#include "rna_internal.h"

#ifdef RNA_RUNTIME

#  include "BLI_string_ref.hh"

#  include "BKE_addon.h"
#  include "BKE_asset_library_custom.h"
#  include "BKE_blender_project.hh"

#  include "BLT_translation.h"

#  include "WM_api.hh"

using namespace blender;

static void rna_BlenderProject_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA * /*ptr*/)
{
  /* TODO evaluate which props should send which notifiers. */
  /* Force full redraw of all windows. */
  WM_main_add_notifier(NC_WINDOW, nullptr);
}

static void rna_BlenderProject_name_get(PointerRNA *ptr, char *value)
{
  const bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  if (!project) {
    value[0] = '\0';
    return;
  }

  strcpy(value, project->project_name().c_str());
}

static int rna_BlenderProject_name_length(PointerRNA *ptr)
{
  const bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  if (!project) {
    return 0;
  }

  return project->project_name().size();
}

static void rna_BlenderProject_name_set(PointerRNA *ptr, const char *value)
{
  bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);

  if (!project) {
    return;
  }

  project->set_project_name(value);
}

static void rna_BlenderProject_root_path_get(PointerRNA *ptr, char *value)
{
  const bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  if (!project) {
    value[0] = '\0';
    return;
  }

  strcpy(value, project->root_path().c_str());
}

static int rna_BlenderProject_root_path_length(PointerRNA *ptr)
{
  const bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  if (!project) {
    return 0;
  }

  return project->root_path().size();
}

static void rna_BlenderProject_root_path_set(PointerRNA * /*ptr*/, const char * /*value*/)
{
  /* Property is not editable, see #rna_BlenderProject_root_path_editable(). */
  BLI_assert_unreachable();
}

static int rna_BlenderProject_root_path_editable(PointerRNA * /*ptr*/, const char **r_info)
{
  /* Path is never editable (setting up a project is an operation), but return a nicer disabled
   * hint. */
  *r_info = N_("Project location cannot be changed, displayed for informal purposes only");
  return 0;
}

static void rna_BlenderProject_asset_libraries_begin(CollectionPropertyIterator *iter,
                                                     PointerRNA *ptr)
{
  bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  ListBase &asset_libraries = project->asset_library_definitions();
  rna_iterator_listbase_begin(iter, &asset_libraries, nullptr);
}

static void rna_BlenderProject_addons_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
  BlenderProject *project = static_cast<BlenderProject *>(ptr->data);
  ListBase *addons_list = BKE_project_addons_get(project);
  rna_iterator_listbase_begin(iter, addons_list, nullptr);
}

static bool rna_BlenderProject_is_dirty_get(PointerRNA *ptr)
{
  const bke::BlenderProject *project = static_cast<bke::BlenderProject *>(ptr->data);
  return project->has_unsaved_changes();
}

static bAddon *rna_BlenderProject_addon_new(ReportList *reports)
{
  BlenderProject *project = BKE_project_active_get();
  if (!project) {
    BKE_report(reports, RPT_ERROR, "No project loaded");
  }

  ListBase *addons_list = BKE_project_addons_get(project);

  bAddon *addon = BKE_addon_new();
  BLI_addtail(addons_list, addon);
  BKE_project_tag_has_unsaved_changes(project);
  return addon;
}

static void rna_BlenderProject_addon_remove(ReportList *reports, PointerRNA *addon_ptr)
{
  BlenderProject *project = BKE_project_active_get();
  if (!project) {
    BKE_report(reports, RPT_ERROR, "No project loaded");
  }

  bAddon *addon = static_cast<bAddon *>(addon_ptr->data);

  ListBase *addons_list = BKE_project_addons_get(project);
  if (BLI_findindex(addons_list, addon) == -1) {
    BKE_report(reports, RPT_ERROR, "Add-on is no longer valid");
    return;
  }
  BLI_remlink(addons_list, addon);
  BKE_addon_free(addon);
  RNA_POINTER_INVALIDATE(addon_ptr);
  BKE_project_tag_has_unsaved_changes(project);
}

#else

static void rna_def_project_addon_collection(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "ProjectAddons");
  srna = RNA_def_struct(brna, "ProjectAddons", nullptr);
  RNA_def_struct_clear_flag(srna, STRUCT_UNDO);
  RNA_def_struct_ui_text(
      srna, "Project Add-ons", "Collection of add-ons, loaded for the current project");

  func = RNA_def_function(srna, "new", "rna_BlenderProject_addon_new");
  RNA_def_function_flag(func, FUNC_NO_SELF | FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Add a new add-on");
  /* return type */
  parm = RNA_def_pointer(func, "addon", "Addon", "", "Add-on data");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove", "rna_BlenderProject_addon_remove");
  RNA_def_function_flag(func, FUNC_NO_SELF | FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove add-on");
  parm = RNA_def_pointer(func, "addon", "Addon", "", "Add-on to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED | PARM_RNAPTR);
  RNA_def_parameter_clear_flags(parm, PROP_THICK_WRAP, ParameterFlag(0));
}

void RNA_def_blender_project(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "BlenderProject", nullptr);
  RNA_def_struct_ui_text(srna, "Blender Project", "");

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(prop,
                                "rna_BlenderProject_name_get",
                                "rna_BlenderProject_name_length",
                                "rna_BlenderProject_name_set");
  RNA_def_property_ui_text(prop, "Name", "The identifier for the project");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(prop, 0, "rna_BlenderProject_update");

  prop = RNA_def_property(srna, "root_path", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(prop,
                                "rna_BlenderProject_root_path_get",
                                "rna_BlenderProject_root_path_length",
                                "rna_BlenderProject_root_path_set");
  RNA_def_property_editable_func(prop, "rna_BlenderProject_root_path_editable");
  RNA_def_property_ui_text(prop, "Location", "The location of the project on disk");

  prop = RNA_def_property(srna, "asset_libraries", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "CustomAssetLibraryDefinition");
  RNA_def_property_collection_funcs(prop,
                                    "rna_BlenderProject_asset_libraries_begin",
                                    "rna_iterator_listbase_next",
                                    "rna_iterator_listbase_end",
                                    "rna_iterator_listbase_get",
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_ui_text(prop, "Asset Libraries", "");

  prop = RNA_def_property(srna, "addons", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_funcs(prop,
                                    "rna_BlenderProject_addons_begin",
                                    "rna_iterator_listbase_next",
                                    "rna_iterator_listbase_end",
                                    "rna_iterator_listbase_get",
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_struct_type(prop, "Addon");
  RNA_def_property_ui_text(prop, "Add-on", "");
  rna_def_project_addon_collection(brna, prop);

  prop = RNA_def_property(srna, "is_dirty", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_funcs(prop, "rna_BlenderProject_is_dirty_get", nullptr);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(
      prop,
      "Dirty",
      "Project settings have changed since read from disk. Save the settings to keep them");
}

#endif
