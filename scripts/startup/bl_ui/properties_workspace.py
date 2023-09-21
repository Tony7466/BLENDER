# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import (
    Panel,
)
from bpy.app.translations import pgettext_iface as iface_

from rna_prop_ui import PropertyPanel


class WorkSpaceButtonsPanel:
    # bl_space_type = 'PROPERTIES'
    # bl_region_type = 'WINDOW'
    # bl_context = ".workspace"

    # Developer note: this is displayed in tool settings as well as the 3D view.
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Tool"


class WORKSPACE_PT_main(WorkSpaceButtonsPanel, Panel):
    bl_label = "Workspace"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        workspace = context.workspace

        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        layout.prop(workspace, "use_pin_scene")
        layout.prop(workspace, "object_mode", text="Mode")


class WORKSPACE_UL_addons_items(bpy.types.UIList):
    def draw_item(self, context, layout, _data, item, icon, _active_data, _active_propname, _index):
        layout.alignment = 'LEFT'
        row = layout.row()
        row.alignment = 'LEFT'
        row.active = context.workspace.use_filter_by_owner
        row.operator(
            "wm.owner_disable" if item.enabled else "wm.owner_enable",
            icon='CHECKBOX_HLT' if item.enabled else 'CHECKBOX_DEHLT',
            text=item.name,
            translate=True,
            emboss=False,
        ).owner_id = item.module


class WORKSPACE_UL_unknown_addons_items(bpy.types.UIList):
    def draw_item(self, context, layout, _data, item, icon, _active_data, _active_propname, _index):
        layout.alignment = 'LEFT'
        row = layout.row()
        row.alignment = 'LEFT'
        row.active = context.workspace.use_filter_by_owner
        row.operator(
            "wm.owner_disable",
            icon='CHECKBOX_HLT',
            text=item.name,
            emboss=False,
        ).owner_id = item.module


class WORKSPACE_PT_addons(WorkSpaceButtonsPanel, Panel):
    bl_label = "Filter Add-ons"
    bl_parent_id = "WORKSPACE_PT_main"

    def draw_header(self, context):
        workspace = context.workspace
        self.layout.prop(workspace, "use_filter_by_owner", text="")

    def draw(self, context):
        layout = self.layout

        bpy.context.workspace.addons.clear()
        bpy.context.workspace.unknown_addons.clear()

        workspace = context.workspace
        prefs = context.preferences

        import addon_utils
        addon_map = {mod.__name__: mod for mod in addon_utils.modules()}
        owner_ids = {owner_id.name for owner_id in workspace.owner_ids}

        for addon in prefs.addons:
            module_name = addon.module
            module = addon_map.get(module_name)
            if module is None:
                continue
            info = addon_utils.module_bl_info(module)
            is_enabled = module_name in owner_ids
            addon = context.workspace.addons.add()
            addon.enabled = is_enabled
            addon.module = module_name
            addon.name = (iface_("%s: %s") % (iface_(info["category"]), iface_(info["name"])))

            if is_enabled:
                owner_ids.remove(module_name)
        row = layout.row()
        layout.template_list(
            "WORKSPACE_UL_addons_items",
            "",
            context.workspace,
            "addons",
            context.workspace,
            "active_addon",
            rows=10)
        # Detect unused
        if owner_ids:
            layout.label(text="Unknown add-ons", icon='ERROR')
            for module_name in sorted(owner_ids):
                addon = context.workspace.unknown_addons.add()
                addon.name = addon.module = module_name
            row = layout.row()
            layout.template_list(
                "WORKSPACE_UL_unknown_addons_items",
                "",
                context.workspace,
                "unknown_addons",
                context.workspace,
                "active_unknown_addon",
                rows=10)


class WORKSPACE_PT_custom_props(WorkSpaceButtonsPanel, PropertyPanel, Panel):
    bl_parent_id = "WORKSPACE_PT_main"

    _context_path = "workspace"
    _property_type = bpy.types.WorkSpace


classes = (
    WORKSPACE_UL_addons_items,
    WORKSPACE_UL_unknown_addons_items,

    WORKSPACE_PT_main,
    WORKSPACE_PT_addons,
    WORKSPACE_PT_custom_props,
)


class WorkspaceAddonPropertyGroup(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()
    enabled: bpy.props.BoolProperty()
    module: bpy.props.StringProperty()


bpy.utils.register_class(WorkspaceAddonPropertyGroup)

bpy.types.WorkSpace.active_addon = bpy.props.IntProperty()
bpy.types.WorkSpace.active_unknown_addon = bpy.props.IntProperty()
bpy.types.WorkSpace.addons = bpy.props.CollectionProperty(type=WorkspaceAddonPropertyGroup)
bpy.types.WorkSpace.unknown_addons = bpy.props.CollectionProperty(type=WorkspaceAddonPropertyGroup)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
