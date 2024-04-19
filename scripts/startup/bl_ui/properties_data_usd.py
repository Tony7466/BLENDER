# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

import os
from typing import *

import bpy
from bpy.props import StringProperty
from bpy.types import (Context, Event, Object, Operator, Panel)
from rna_prop_ui import PropertyPanel


def usd_stage_poll(cls, context:Context) -> bool:
    ob = context.object
    return ob and ob.type == "USDSTAGE"


class OB_OT_USDStage_OpenStageFile(Operator):
    """Provide a dialogue to choose a USDStage file to open"""

    bl_idname = "usdstage.open_stage_file"
    bl_label = "Open USDStage File"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    filepath:       StringProperty(subtype="FILE_PATH")
    filter_glob:    StringProperty(default="*.usd;*.usda;*.usdc;*.usdz", options={"HIDDEN"})

    @classmethod
    def poll(cls, context:Context) -> bool:
        return usd_stage_poll(cls, context)

    def execute(self, context:Context) -> Set[str]:
        ob = context.object
        ob.data.filepath = self.filepath
        self.report({"INFO"}, f"Loaded USD Stage: {ob.data.filepath}")
        return {"FINISHED"}

    def invoke(self, context:Context, event:Event) -> Set[str]:
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class USDButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        ##!TODO(kiki): should there be a context.usd bit for Collections?
        engine = context.engine
        return context.object and context.object.type == "USDSTAGE" and (engine in cls.COMPAT_ENGINES)


class DATA_PT_context_usd(USDButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_EEVEE', 'BLENDER_WORKBENCH'}

    def draw(self, context:Context):
        layout = self.layout

        ob = context.object
        mesh = context.mesh
        space = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif mesh:
            layout.template_ID(space, "pin_id")


class DATA_PT_usd_stage_settings(USDButtonsPanel, Panel):
    bl_label = "Stage Settings"
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_EEVEE', 'BLENDER_WORKBENCH'}

    def draw(self, context):
        layout = self.layout
        ob     = context.object
        stage  = ob.data
        space  = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif stage:
            layout.template_ID(space, "pin_id")

        row = layout.row(align=True)
        row.label(text="File Path")
        row = row.row(align=True)
        row.prop(stage, "filepath", text="")
        row.operator(operator="usdstage.open_stage_file", text="", icon="FILE_FOLDER")

        row = layout.row(align=True)
        row.label(text="Root Prim")
        row = row.row(align=True)
        row.prop_search(stage, "root_prim_path", stage, "object_paths", text="")


class DATA_PT_usd_stage_frame_control(USDButtonsPanel, Panel):
    bl_label = "Frame Control"
    # bl_options = {'DEFAULT_CLOSED'}
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_EEVEE', 'BLENDER_WORKBENCH'}

    def draw(self, context):
        layout = self.layout.column()
        stage = context.object.data
        row = self.layout.column()
        row.prop(stage, "override_frame")
        sub_col = row.row()
        sub_col.prop(stage, "frame")
        sub_col.enabled = stage.override_frame
        layout.prop(stage, "frame_offset")


class DATA_PT_usd_stage_active_prim(USDButtonsPanel, Panel):
    bl_label = "Active Prim"
    # bl_options = {'DEFAULT_CLOSED'}
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_EEVEE', 'BLENDER_WORKBENCH'}

    @classmethod
    def poll(cls, context):
        return context.object and context.object.type == "USDSTAGE" and len(context.object.data.active_prim_path)

    def draw(self, context):
        layout = self.layout
        stage = context.object.data
        row = self.layout.row()
        row.label(text="Path:")
        row.label(text=stage.active_prim_path)
        row = self.layout.row()
        row.prop(stage, "active_prim_type", text="Type")
        row = self.layout.row()
        row.prop(stage, "active_prim_purpose", text="Purpose")
        row = self.layout.row()
        row.prop(stage, "active_prim_variant", text="Variant")


class DATA_PT_custom_props_usd(USDButtonsPanel, PropertyPanel, Panel):
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_EEVEE', 'BLENDER_WORKBENCH'}
    _context_path = "object.data"
    _property_type = bpy.types.USDStage


classes = (
    OB_OT_USDStage_OpenStageFile,
    DATA_PT_usd_stage_settings,
    DATA_PT_usd_stage_frame_control,
    DATA_PT_usd_stage_active_prim,
    DATA_PT_context_usd,
    DATA_PT_custom_props_usd,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
