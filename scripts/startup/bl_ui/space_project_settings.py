# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Header, Menu, Panel
from bpy.app.translations import pgettext_iface as iface_
from bl_ui.utils import CenterAlignMixIn
from bl_ui.addons_ui import AddonsUI


# -----------------------------------------------------------------------------
# Main Header

class PROJECTSETTINGS_HT_header(Header):
    bl_space_type = 'PROJECT_SETTINGS'

    @staticmethod
    def draw_buttons(layout, context):
        project = context.project

        layout.operator_context = 'EXEC_AREA'

        is_dirty = project and project.is_dirty

        # Show '*' to let users know the settings have been modified.
        layout.operator(
            "project.save_settings",
            text=iface_("Save Settings") + (" *" if is_dirty else ""),
            translate=False,
        )

    def draw(self, context):
        layout = self.layout
        layout.operator_context = 'EXEC_AREA'

        layout.template_header()

        PROJECTSETTINGS_MT_editor_menus.draw_collapsible(context, layout)

        layout.separator_spacer()

        self.draw_buttons(layout, context)


# -----------------------------------------------------------------------------
# Main Navigation Bar

class PROJECTSETTINGS_PT_navigation_bar(Panel):
    bl_label = "Project Settings Navigation"
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'NAVIGATION_BAR'
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout

        space_data = context.space_data
        project = context.project

        col = layout.column()
        col.enabled = project is not None

        col.scale_x = 1.3
        col.scale_y = 1.3
        col.prop(space_data, "active_section", expand=True)


class PROJECTSETTINGS_MT_editor_menus(Menu):
    bl_idname = "PROJECTSETTINGS_MT_editor_menus"
    bl_label = ""

    def draw(self, _context):
        layout = self.layout
        layout.menu("PROJECTSETTINGS_MT_view")


class PROJECTSETTINGS_MT_view(Menu):
    bl_label = "View"

    def draw(self, _context):
        layout = self.layout

        layout.menu("INFO_MT_area")


class PROJECTSETTINGS_MT_advanced_operations(Menu):
    bl_label = "Advanced Project Settings Operations"

    def draw(self, _context):
        layout = self.layout

        layout.operator("project.delete_setup")


class PROJECTSETTINGS_PT_save_project_settings(Panel):
    bl_label = "Save Project Settings"
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'EXECUTE'
    bl_options = {'HIDE_HEADER'}

    @classmethod
    def poll(cls, context):
        # Hide when header is visible
        for region in context.area.regions:
            if region.type == 'HEADER' and region.height <= 1:
                return True

        return False

    def draw(self, context):
        layout = self.layout.row()
        layout.operator_context = 'EXEC_AREA'

        layout.menu("PROJECTSETTINGS_MT_advanced_operations", text="", icon='COLLAPSEMENU')

        PROJECTSETTINGS_HT_header.draw_buttons(layout, context)


class PROJECTSETTINGS_PT_no_project(Panel):
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'WINDOW'
    # Special hardcoded context.
    bl_context = "no_project"
    bl_label = "No Project"
    bl_options = {'HIDE_HEADER'}

    @classmethod
    def poll(cls, context):
        return (context.project is None)

    def draw(self, context):
        layout = self.layout

        layout.label(text="No active project.", icon='INFO')
        col = layout.column(align=True)
        col.label(text="Open/store a file inside of a project directory, or set up a new project")
        col.label(text="by choosing a project directory.")

        row = layout.row()
        split = row.split(factor=0.3)
        split.operator("project.new", text="Set up Project...")


class PROJECTSETTINGS_PT_setup(CenterAlignMixIn, Panel):
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'WINDOW'
    bl_context = "general"
    bl_label = "Setup"
    bl_options = {'HIDE_HEADER'}

    def draw_centered(self, context, layout):
        project = context.project

        layout.prop(project, "name")
        layout.prop(project, "root_path", text="Location")


class PROJECTSETTINGS_PT_addons(Panel):
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'WINDOW'
    bl_label = "Add-ons"
    bl_options = {'HIDE_HEADER'}
    bl_context = "addons"

    def draw(self, context):

        layout = self.layout

        # TODO temporary duplicated UI code.

        # AddonsUI.draw(context, layout)

        layout.operator("preferences.project_addon_install", icon='IMPORT', text="Install...")

        import os
        import addon_utils
        from bpy.app.translations import (
            pgettext_iface as iface_,
            pgettext_tip as tip_,
        )
        project = context.project
        used_ext = {ext.module for ext in project.addons}

        project_addon_dirs = tuple(
            p for p in (
                [os.path.join(bpy.utils.script_path_project(), "addons")]
            )
            if p
        )

        addons = [
            (mod, addon_utils.module_bl_info(mod))
            for mod in addon_utils.modules(refresh=False) if mod.__file__.startswith(project_addon_dirs)
        ]

        col = layout.column()

        for mod, info in addons:
            module_name = mod.__name__

            is_enabled = module_name in used_ext

            col_box = col.column()
            box = col_box.box()
            colsub = box.column()
            row = colsub.row(align=True)

            row.operator(
                "preferences.addon_expand",
                icon='DISCLOSURE_TRI_DOWN' if info["show_expanded"] else 'DISCLOSURE_TRI_RIGHT',
                emboss=False,
            ).module = module_name

            props = row.operator(
                "preferences.addon_disable" if is_enabled else "preferences.addon_enable",
                icon='CHECKBOX_HLT' if is_enabled else 'CHECKBOX_DEHLT', text="",
                emboss=False,
            )
            props.module = module_name
            props.owner = 'PROJECT'

            sub = row.row()
            sub.active = is_enabled
            sub.label(text=iface_("%s: %s") % (iface_(info["category"]), iface_(info["name"])))
            if info["show_expanded"]:
                if info["description"]:
                    split = colsub.row().split(factor=0.15)
                    split.label(text="Description:")
                    split.label(text=tip_(info["description"]))

                split = colsub.row().split(factor=0.15)

                props = split.operator(
                    "preferences.addon_remove", text="Remove From Project", icon='CANCEL',
                )
                props.module = module_name
                props.owner = 'PROJECT'



class PROJECTSETTINGS_PT_asset_libraries(Panel):
    bl_space_type = 'PROJECT_SETTINGS'
    bl_region_type = 'WINDOW'
    bl_context = "asset_libraries"
    bl_label = "Asset Libraries"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = False
        layout.use_property_decorate = False

        project = context.project

        box = layout.box()
        split = box.split(factor=0.35)
        name_col = split.column()
        path_col = split.column()

        row = name_col.row(align=True)  # Padding
        row.separator()
        row.label(text="Name")

        row = path_col.row(align=True)  # Padding
        row.separator()
        row.label(text="Path")

        for i, library in enumerate(project.asset_libraries):
            row = name_col.row()
            row.alert = not library.name
            row.prop(library, "name", text="")

            row = path_col.row()
            subrow = row.row()
            subrow.alert = not library.path
            subrow.prop(library, "path", text="")
            row.operator("project.custom_asset_library_remove", text="", icon='X', emboss=False).index = i

        row = box.row()
        row.alignment = 'RIGHT'
        row.operator("project.custom_asset_library_add", text="", icon='ADD', emboss=False)


classes = (
    PROJECTSETTINGS_HT_header,
    PROJECTSETTINGS_MT_editor_menus,
    PROJECTSETTINGS_MT_view,
    PROJECTSETTINGS_MT_advanced_operations,
    PROJECTSETTINGS_PT_navigation_bar,
    PROJECTSETTINGS_PT_save_project_settings,
    PROJECTSETTINGS_PT_no_project,
    PROJECTSETTINGS_PT_setup,
    PROJECTSETTINGS_PT_addons,
    PROJECTSETTINGS_PT_asset_libraries,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
